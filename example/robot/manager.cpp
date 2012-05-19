#include <QtCore/QCoreApplication>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <time.h>
#include <QDebug>
#include <iostream>
#include <cmath>
#include "manager.h"
#include "algorithmsolver.h"

Manager::Manager(QObject *parent, QString configurationFile) :
    QObject(parent)
{
    robot = new Robot();

    configurationLoaded = false;
    loadConfiguration(configurationFile);

    path = QVector<QPair<int, int> >();
    targetNum = 0;

    sleep(2);
}

void Manager::run()
{
    if (configurationLoaded) {
        if(robot->getState() == Started)
            action();
        QTimer::singleShot(PERFORM_ACTION_FREQUENCY, this, SLOT(run()));
    } else {
        std::cout << "Robot configuration is not loaded. " <<
                     "Put robot's profile and this binary to /robots directory\n";
        emit stop();
    }
}

void Manager::action()
{
    if (path.size() == 0) {
        objects = robot->whoIsThere(100500, 100500, 1488);

        if (objects.size() == 0)
            return;

        std::cout << "Robot sees " << objects.size() << " objects\n";

        const int size = objects.size() + 1;    //add robot's position
        if (size >= 2) {
            MessageObject me;
            me.coordX = robot->getCoords().first;
            me.coordY = robot->getCoords().second;
            objects.push_back(me);

            double **costMatrix = new double*[size];
            for (int i = 0; i < size; i++) {
                costMatrix[i] = new double[size];
            }

            for (int i = 0; i < size; i++) {
                for (int j = 0; j < size; j++) {
                    if (i == j) {
                        costMatrix[i][j] = std::numeric_limits<double>::infinity();
                    } else {
                        costMatrix[i][j] = sqrt(pow(static_cast<int>(objects.at(i).coordX
                                                                     - objects.at(j).coordX), 2)
                                                + pow(static_cast<int>(objects.at(i).coordY
                                                                       - objects.at(j).coordY), 2));
                    }
                }
            }

            AlgorithmSolver *solver = new AlgorithmSolver(costMatrix, size);


//            path.push_back(QPair<int, int>(4, 6));
//            path.push_back(QPair<int, int>(1, 5));
//            path.push_back(QPair<int, int>(5, 10));
//            path.push_back(QPair<int, int>(10, 1));
//            path.push_back(QPair<int, int>(6, 9));
//            path.push_back(QPair<int, int>(8, 7));
//            path.push_back(QPair<int, int>(9, 8));
//            path.push_back(QPair<int, int>(0, 4));
//            path.push_back(QPair<int, int>(3, 2));
//            path.push_back(QPair<int, int>(2, 0));
//            path.push_back(QPair<int, int>(7, 3));

            for (int i = 0; i < size; i++) {
                for (int j = 0; j < size; j++) {
                    printf("%7.2f | ", costMatrix[i][j]);
                }
                printf("\n");
            }

//            exit(0);
            path = solver->solve();
            qDebug() << "Path: " << path;
            //delete solver;

            targetNum = size - 1;
            for (int i = 0; i < path.size(); i++) {
                if (path.at(i).first == size - 1) {
                    targetNum = path.at(i).second;
                    break;
                }
            }

            qDebug() << targetNum << objects.at(targetNum).coordX << objects.at(targetNum).coordY;

            robot->setParameter(0, objects.at(targetNum).coordX);
            robot->setParameter(1, objects.at(targetNum).coordY);

            qDebug() << robot->getParameter(0) << robot->getParameter(1);

        } else if (size == 1) {
            robot->setParameter(0, objects.at(0).coordX);
            robot->setParameter(1, objects.at(0).coordY);

        } else
            return;
    } else if (path.size() > 0 && (robot->getParameter(0) + robot->getSize() / 2 >= robot->getCoords().first
                && robot->getParameter(0) - robot->getSize() / 2 <= robot->getCoords().first)
               &&
               (robot->getParameter(1) + robot->getSize() / 2 >= robot->getCoords().second
                && robot->getParameter(1) - robot->getSize() / 2 <= robot->getCoords().second)) {

        //qDebug() << robot->getParameter(0) << robot->getCoords().first;
        //qDebug() << robot->getParameter(1) << robot->getCoords().second;

        for (int i = 0; i < path.size(); i++) {
            if (path.at(i).first == targetNum && path.at(i).second != objects.size() - 1) {
                targetNum = path.at(i).second;
                break;
            } else if (path.at(i).first == targetNum && path.at(i).second == objects.size() - 1) {
                return;
            }
        }

        robot->setParameter(0, objects.at(targetNum).coordX);
        robot->setParameter(1, objects.at(targetNum).coordY);
    }

    double x1 = robot->getCoords().first;
    double y1 = robot->getCoords().second;
    double x2 = robot->getParameter(0);
    double y2 = robot->getParameter(1);
    double y;
    double x;

    double part;
    part = fabs(robot->getParameter(2)) / sqrt(pow((x2 - x1) / (y2 - y1), 2) + 1);
    if (part + y1 < y2)
        y = part + y1;
    else
        y = y1 - part;
    x = (x2 - x1) * (y - y1) / (y2 - y1) + x1;

    if (isnan(x) != 0)
        x = (x2 > x1) ? (x1 + robot->getParameter(2)) : (x1 - robot->getParameter(2));

//    QFile file("log.txt");
//    file.open(QIODevice::Append | QIODevice::WriteOnly | QIODevice::Text);
//    QTextStream out(&file);
//    //out << QString("Move to: %1 %2 | %3 %4").arg(x).arg(y).arg(static_cast<int>(x)).arg(static_cast<int>(y));
//    out << QString("Target id: %1 | Robot params: %2 %3 | Target coords: %4 %5\n").
//           arg(targetNum).arg(robot->getParameter(0)).arg(robot->getParameter(1)).
//           arg(objects.at(targetNum).coordX).arg(objects.at(targetNum).coordY);
//    file.close();

    robot->move(static_cast<int>(x), static_cast<int>(y));

}

void Manager::loadConfiguration(QString configurationFile)
{  
    // Load file contents (without commented strings) to configStringList
    QFile config(QString("robots/") + configurationFile);
    QStringList configStringList = QStringList();
    if (!config.open(QFile::ReadOnly)) {
        std::cout << "Cannot open configuration file\n";
        return;
    }
    QTextStream stream (&config);
    QString line;
    while(!stream.atEnd()) {
        line = stream.readLine();
        if (!line.contains(QRegExp("^(//)")))
            configStringList.append(line);
    }
    config.close();

    // Check if port in filename is equals to port in file body
    int portFilename = configurationFile.left(4).toInt();
    if (portFilename == 0 || portFilename != configStringList.at(1).toInt()) {
        std::cout << "Ports in profile name and profile body aren't equals\n";
        return;
    }

    // Check start position
    QString pos = configStringList.at(2);
    if (!pos.contains(QRegExp("^(\\d)+;(\\d)+$"))) {
        std::cout << "Invalid start position\n";
        return;
    }
    bool ok = true;
    int x = pos.split(";").at(0).toInt(&ok);
    if (!ok) {
        std::cout << "Invalid start position\n";
        return;
    }
    int y = pos.split(";").at(1).toInt(&ok);
    if (!ok) {
        std::cout << "Invalid start position\n";
        return;
    }

    // Check if size is a number and is over than zero
    int size = configStringList.at(3).toInt();
    if (size <= 0) {
        std::cout << "Invalid size\n";
        return;
    }

    int type = configStringList.at(4).toInt(&ok);
    if (!ok || (type != 0 && type != 1)) {
        std::cout << "Invalid robot type";
        return;
    }

    // Check visibility radius
    int visibilityRadius = configStringList.at(5).toInt(&ok);
    if (!ok || visibilityRadius < 0) {
        std::cout << "Invalid visibilty radius";
        return;
    }

    // Check intersection type
    QString intersection = configStringList.at(8);
    if (intersection != "0" && intersection != "1" && intersection != "2") {
        std::cout << "Invalid intersection type\n";
        return;
    }

    // Check orientation
    double orientation = configStringList.at(9).toDouble(&ok);
    if (!ok) {
        std::cout << "Invalid orientation\n";
        return;
    }

    // Check color
    QColor color = QColor(configStringList.at(10));
    if (!color.isValid()) {
        std::cout << "Invalid color\n";
        return;
    }

    // Check all custom parameters
    std::pair<std::string, double> *parameters =
            new std::pair<std::string, double>[CUSTOM_PARAMETERS_QUANTITY];
    for (int i = 0; i < CUSTOM_PARAMETERS_QUANTITY; i++) {
        QString line = configStringList.at(11+i);
        if (!line.contains(QRegExp("^(\\d|\\.)+;(\\w|\\s)+$"))) {
            std::cout << "Invalid parameter " << i+1 << " \n";
            return;
        }
        double value = line.left(line.indexOf(QString(";"))).toDouble(&ok);
        if (!ok) {
            std::cout << "Invalid value of parameter" << i+1 << "\n";
            return;
        }
        std::string name = line.mid(line.indexOf(QString(";")) + 1).toStdString();
        parameters[i] = std::pair<std::string, double>(name, value);
    }

    robot->setPortNumber(portFilename);
    robot->move(x, y);
    robot->turn(orientation);
    robot->changeDiameter(size);
    robot->changeColor(color.red(), color.green(), color.blue());

    for (int i = 0; i < CUSTOM_PARAMETERS_QUANTITY; i++) {
        robot->setParameter(i, parameters[i].second);
    }

    robot->setIntersection(static_cast<Intersection>(intersection.toInt()));

    configurationLoaded = true;
}

/* Limit line length to 100 characters; highlight 99th column
 * vim: set textwidth=100 colorcolumn=-1:
 */
