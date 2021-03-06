#include <QtCore/QCoreApplication>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <iostream>
#include <time.h>
#include <cmath>
#include "manager.h"

Manager::Manager(QObject *parent, QString configurationFile,
                 std::pair<unsigned int, unsigned int> mapSize) :
    QObject(parent)
{
    this->mapSize = std::pair<unsigned int, unsigned int>(mapSize.first * REAL_PIXEL_SIZE,
                                                          mapSize.second * REAL_PIXEL_SIZE);
    configurationLoaded = false;
    loadConfiguration(configurationFile);
}

void Manager::run()
{
    if (configurationLoaded) {

        checkForStateChanges();
        if (EnvObject::getState() == Started)
            action();
        QTimer::singleShot(PERFORM_ACTION_FREQUENCY, this, SLOT(run()));
    } else {
        std::cout << "Environment configuration is not loaded. " <<
                     "Put environment's profile and this binary to /robots directory\n";
        emit stop();
    }
}

void Manager::action()
{
    foreach (EnvObject* object, envObjects) {
        //TODO: implement action for each object

        if (object->getCoords().first < mapSize.first - 2000
                && object->getCoords().second < mapSize.second - 2000
                ) {

//            if (!object->move(object->getCoords().first,
//                              object->getCoords().second)) {
//                object->changeDiameter(0);
//            }

        }
    }
}

void Manager::loadConfiguration(QString configurationFile)
{
    srand(static_cast<unsigned int>(time(0)));

    // Load file contents (without commented strings) to configStringList
    QFile config(QString("robots/") + configurationFile);
    QStringList configStringList = QStringList();
    if (!config.open(QFile::ReadOnly)) {
        std::cout << "Cannot open configuration file for the environment";
        return;
    }
    QTextStream stream (&config);
    QString line;
    while(!stream.atEnd()) {
        line = stream.readLine();
        if (!line.contains(QRegExp("^(//)")) && !line.isEmpty())
            configStringList.append(line);
    }
    config.close();

    // check launch command
    if (configStringList.at(0).isEmpty()) {
        std::cout << "Launch command is empty!";
        return;
    }

    // check port
    int portFilename = configurationFile.left(4).toInt();
    if (portFilename == 0 || portFilename != configStringList.at(1).toInt()) {
        std::cout << "Ports in filename and file body aren't equal!";
        return;
    }
    EnvObject::setPortNumber(portFilename);

    // start parsing each object
    const int parametersQuantityPerObject = 8;      // See AI-simulator wiki
    QVector<int> indexes = QVector<int>();
    for (int obj = 0; obj < ENV_OBJECTS; obj++) {

        if (configStringList.size() < 2 + (obj+1) * parametersQuantityPerObject)
            break;
        QStringList objectParams = QStringList();
        for (int i = 0; i < parametersQuantityPerObject; i++)
            objectParams.push_back(configStringList.at(2 + obj * parametersQuantityPerObject + i));
        bool ok = true;

        // Check object id
        int index = objectParams.at(0).toInt(&ok);
        if (!ok || index < 0) {
            std::cout << "Id must be non-negative integer number (object " << obj << " )";
            return;
        }
        if (indexes.contains(index)) {
            std::cout << "Object with same id already exists (object " << obj << " )";
            return;
        }

        // Check if size is a number and is over than zero
        int size = objectParams.at(2).toInt();
        if (size <= 0) {
            std::cout << "Invalid size (object " << obj << " )";
            return;
        }

        // Check object start position
        QString pos = objectParams.at(1);
        if (!pos.contains(QRegExp("^(\\d)+;(\\d)+$")) && pos != QString("-1;-1")) {
            std::cout << "Invalid start position (object " << obj << " )";
            return;
        }
        int x, y;
        if (pos != QString("-1;-1"))
        {
            x = pos.split(";").at(0).toInt(&ok);
            if (!ok) {
                std::cout << "Invalid start position (object " << obj << " )";
                return;
            }
            y = pos.split(";").at(1).toInt(&ok);
            if (!ok) {
                std::cout << "Invalid start position (object " << obj << " )";
                return;
            }
            if (x < 0 || y < 0 || x >= mapSize.first
                    || y >= mapSize.second) {
                std::cout << "Start position is out of the map (object " << obj << " )";
                return;
            }
        } else {

            x = size + rand() % (mapSize.first - 4000);
            y = size + rand() % (mapSize.second - 4000);
            std::cout << "Object " << obj <<
                         " receives random coordinates ( " << x << ", " << y << " )";
        }

        // Check intersection type
        QString intersection = objectParams.at(3);
        if (intersection != "0" && intersection != "1" && intersection != "2") {
            std::cout << "Invalid intersection type (object " << obj << " )";
            return;
        }

        bool movable;
        if (objectParams.at(4) == QString("0"))
            movable = false;
        else if (objectParams.at(4) == QString("1"))
            movable = true;
        else {
            std::cout << "Movable parameter can receive only 0 or 1 (object " << obj << " )";
            return;
        }

        // Check orientation
        double orientation = objectParams.at(5).toDouble(&ok);
        if (!ok || orientation < 0) {
            std::cout << "Invalid orientation (object " << obj << " )";
            return;
        }

        int velocity = objectParams.at(6).toInt();
        if (velocity <= 0) {
            std::cout << "Invalid velocity (object " << obj << " )";
            return;
        }

        // Check color
        QColor color = QColor(objectParams.at(7));
        if (!color.isValid()) {
            std::cout << "Invalid color (object " << obj << " )";
            return;
        }

        indexes.push_back(index);
        EnvObject *envObject = new EnvObject(index, movable,
                                             static_cast<Intersection>(intersection.toInt()),
                                             velocity, std::pair<int, int>(x, y));
        envObject->changeDiameter(size);
        envObject->turn(orientation);
        envObject->changeColor(color.red(), color.green(), color.blue());
        envObjects.push_back(envObject);
    }

    configurationLoaded = true;
}

void Manager::checkForStateChanges()
{
    Message *msg = EnvObject::getNetwork()->receive();
    if (msg && msg->type == MsgStart) {
        EnvObject::setState(Started);
    } else if (msg && msg->type == MsgPause) {
        EnvObject::setState(Paused);
    } else if (msg && msg->type == MsgBump) {

        MessageBump *m = static_cast<MessageBump *>(msg);
        unsigned int object = m->envObjID;
        for (unsigned int i = 0; i < envObjects.size(); i++) {
            if ((envObjects.at(i) != NULL) && ((envObjects.at(i)->getObjectId() + 1) == object)) {
                envObjects.at(i)->receiveBump(m);
            }
        }
    }
    if (msg) delete msg;
}

/* Limit line length to 100 characters; highlight 99th column
 * vim: set textwidth=100 colorcolumn=-1:
 */
