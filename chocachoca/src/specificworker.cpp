/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx) {
    this->startup_check_flag = startup_check;
    // Uncomment if there's too many debug messages
    // but it removes the possibility to see the messages
    // shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker() {
    std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params) {
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






    return true;
}

void SpecificWorker::initialize(int period) {
    std::cout << "Initialize worker" << std::endl;
    this->Period = period;
    if (this->startup_check_flag) {
        this->startup_check();
    } else {
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460, 480, 0, 100, QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
        timer.start(Period);

    }

}

void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
    }

    decltype(ldata) vectorPoints;

    std::copy_if(
            ldata.begin(),
            ldata.end(),
            std::back_inserter(vectorPoints),
            [](auto a) { return a.z < 2000; });
//                [](auto a){return true;});
    if (vectorPoints.empty()) return;

    int offset = vectorPoints.size() / 2 - vectorPoints.size() / 5;

    draw_lidar(vectorPoints, viewer, *(vectorPoints.begin() + (int)(vectorPoints.size() * 0.4)), *(vectorPoints.begin()  + (int)(vectorPoints.size() * 0.6)));
    //modos
    ///control


//        int offset = 0;

    auto primer_elemento = *std::min_element(vectorPoints.begin() + offset,
                                             vectorPoints.end() - offset,
                                             [](auto &a, auto &b) {
                                                 return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);
                                             });

//    qInfo() << sqrt(primer_elemento.x * primer_elemento.x +
//                    primer_elemento.y * primer_elemento.y +
//                    primer_elemento.z * primer_elemento.z);

    switch (estado){
        case Estado::FOLLOW_WALL:
            estado = follow_wall(*(vectorPoints.begin() + vectorPoints.size() * 0.4), *(vectorPoints.begin()  + vectorPoints.size() * 0.6));
            break;
        case Estado::STRAIGHT_LINE:
            qInfo() << ldata.size();
            estado = straight_line(primer_elemento);
    }
//    straight_line(primer_elemento);

}

SpecificWorker::Estado SpecificWorker::follow_wall(RoboCompLidar3D::TPoint first_point, RoboCompLidar3D::TPoint last_point){
#define THRESHOLD 0.15

    int y_diff = last_point.y-first_point.y;
    int x_diff = last_point.x-first_point.x;


    double angle = std::abs(std::atan2(y_diff, x_diff));
    qInfo() << "ANGULO: " << angle;
    if (angle > (std::numbers::pi/2) + THRESHOLD || angle < (std::numbers::pi/2) + -THRESHOLD) {
        // STOP the robot && START
        omnirobot_proxy->setSpeedBase(0, 0, 0.5);
        return Estado::FOLLOW_WALL;
    } else {
        //start the robot
        try {

            return Estado::STRAIGHT_LINE;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
#undef THRESHOLD
}

SpecificWorker::Estado SpecificWorker::straight_line(auto primer_elemento){
    const float MIN_DISTANCE = 1000;
    if (std::hypot(primer_elemento.x, primer_elemento.y) < MIN_DISTANCE) {
        // STOP the robot && START
        return Estado::FOLLOW_WALL;
    } else {
        //start the robot
        try {

            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);
            return  Estado::STRAIGHT_LINE;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
}


int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints& points, AbstractGraphicViewer *scene, RoboCompLidar3D::TPoint& first, RoboCompLidar3D::TPoint& last) {
    static std::vector<QGraphicsItem *> borrar;
//    std::for_each(borrar.begin(),borrar.end(),[this](auto a){viewer->scene.removeItem(a);});


    for (auto &p: borrar) {
        viewer->scene.removeItem(p);
        delete p;
    }


    borrar.clear();
    float i = 0;
    for (const auto &p: points) {

        QColor color;

//        qInfo() << "\t First: " << first.x << " " << first.y;
//qInfo() << "\t P: " << p.x << " " << p.y;

        if (p.x == first.x && p.y == first.y)
            color.setNamedColor("Blue");
        else if (p.x == last.x && p.y == last.y)
            color.setNamedColor("Magenta");
        else
            color.setNamedColor(i/points.size() > 0.4 && i/points.size() < 0.6 ?"Green":"Red");

        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }
}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

