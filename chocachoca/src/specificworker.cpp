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

std::vector<RoboCompLidar3D::TPoints> SpecificWorker::filterLidarPoints(std::vector<RoboCompLidar3D::TPoints>& points) {
    const float Z_MAXHEIGHT = 2000;
    std::vector<RoboCompLidar3D::TPoints> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [](auto a) { return a.z < Z_MAXHEIGHT; });

    return vectorPoints;
}

RoboCompLidar3D::TPoint SpecificWorker::closestElement(std::iterator& begin, std::iterator& end) {
    return *std::min_element(begin,end,
                            [](auto &a, auto &b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
}

void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
    }

    auto vectorPoints = filterLidarPoints(ldata);

    if (vectorPoints.empty()) return;

    int offset = vectorPoints.size() / 2 - vectorPoints.size() / 5;
    auto primer_elemento = closestElement(vectorPoints.begin() + offset, vectorPoints.end() - offset);


//    qInfo() << sqrt(primer_elemento.x * primer_elemento.x +
//                    primer_elemento.y * primer_elemento.y +
//                    primer_elemento.z * primer_elemento.z);
    draw_lidar(vectorPoints, viewer, *(vectorPoints.begin() + (int)(vectorPoints.size() * 0.4)), *(vectorPoints.begin()  + (int)(vectorPoints.size() * 0.6)));

    switch (estado){
        case Estado::FOLLOW_WALL:
            estado = follow_wall(vectorPoints);
            break;
        case Estado::STRAIGHT_LINE:
            qInfo() << ldata.size();
            estado = straight_line(primer_elemento);
    }
//    straight_line(primer_elemento);

}

SpecificWorker::Estado SpecificWorker::follow_wall(RoboCompLidar3D::TPoints points){
    auto first_point = points[points.size()/2];
    auto last_point = points[points.size()/2 + CENTRAL_POINTS_DIFF];

    int y_diff = (last_point.y-first_point.y);
    int x_diff = last_point.x-first_point.x;


    double angle = std::abs(std::atan2(y_diff, x_diff));
    qInfo() << "ANGULO: " << angle;
    qInfo() << "First: X: " << first_point.x << "  Y: " << first_point.y;
    qInfo() << "Last: X: " << last_point.x << "  Y: " << last_point.y;
    qInfo() << "DIFF: X: " << x_diff << "  Y: " << y_diff;
    //if (angle > (std::numbers::pi/2) + THRESHOLD || angle < (std::numbers::pi/2) + -THRESHOLD) {
    if (std::abs(x_diff) > 10) {
        // STOP the robot && START
        omnirobot_proxy->setSpeedBase(0, 0, 0.5);
        return Estado::FOLLOW_WALL;
    } else {
        //start the robot
        try {
            omnirobot_proxy->setSpeedBase(0, 0, 0.0);
            return Estado::STRAIGHT_LINE;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
        }
    }
}

SpecificWorker::Estado SpecificWorker::straight_line(auto primer_elemento){
    const float MIN_DISTANCE = 1000;
    if (std::hypot(primer_elemento.x, primer_elemento.y) < MIN_DISTANCE) {
        // STOP the robot && START
        omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);

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

    auto p = points[points.size()/2];
    auto point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Magenta")),
                                       QBrush(QColor("Magenta")));
    point->setPos(p.x, p.y);
    borrar.push_back(point);


    p = points[points.size()/2 + CENTRAL_POINTS_DIFF];
    point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Yellow")),
                                       QBrush(QColor("Yellow")));
    point->setPos(p.x, p.y);
    borrar.push_back(point);


}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
