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

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterLidarPoints(const RoboCompLidar3D::TPoints& points) {
    const float Z_MAXHEIGHT = 2000;
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [Z_MAXHEIGHT](const auto& a) { return a.z < Z_MAXHEIGHT; });

    return vectorPoints;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterForwardPoints(const std::vector<RoboCompLidar3D::TPoint>& points) {
    const float FORWARD_ANGLE = 10 * (M_PI / 180);
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
        points.begin(),
        points.end(),
        std::back_inserter(vectorPoints),
        [FORWARD_ANGLE](const auto& a) {
            if (a.y < 0) return false;
            auto angle = std::atan2(a.y, a.x);
            return std::abs(angle-M_PI/2) < FORWARD_ANGLE;
        });
    return vectorPoints;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::filterClosePoints(const std::vector<RoboCompLidar3D::TPoint>& points) {
    const float MAX_DISTANCE = 1500;
    std::vector<RoboCompLidar3D::TPoint> vectorPoints;

    std::copy_if(
            points.begin(),
            points.end(),
            std::back_inserter(vectorPoints),
            [MAX_DISTANCE](const auto& p) {
                return std::hypot(p.x,p.y,p.z) < MAX_DISTANCE;
            });
    return vectorPoints;
}

RoboCompLidar3D::TPoint SpecificWorker::closestElement( const std::vector<RoboCompLidar3D::TPoint>::iterator& begin,
                                                        const std::vector<RoboCompLidar3D::TPoint>::iterator& end) {
    return *std::min_element(begin,end,
                            [](const auto &a, const auto &b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
}

void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return;
    }

    auto vectorPoints = filterLidarPoints(ldata);

    auto forwardPoints = filterForwardPoints(vectorPoints);
    auto closePoints = filterClosePoints(vectorPoints);

    if (vectorPoints.empty() || forwardPoints.empty()) return;

    int offset = vectorPoints.size() / 2 - vectorPoints.size() / 5;
    auto primer_elemento = closestElement(forwardPoints.begin(), forwardPoints.end());

//    qInfo() << sqrt(primer_elemento.x * primer_elemento.x +
//                    primer_elemento.y * primer_elemento.y +
//                    primer_elemento.z * primer_elemento.z);
    draw_lidar(vectorPoints, viewer, forwardPoints, closePoints);

    switch (estado){
        case Estado::TURN:
            if (closePoints.size() < CENTRAL_POINTS_DIFF*2) {
                estado = Estado::STRAIGHT_LINE;
                break;
            }
            estado = turn(closePoints);
            break;
        case Estado::STRAIGHT_LINE:
            qInfo() << ldata.size();
            estado = straight_line(primer_elemento);
            break;
        case Estado::FOLLOW_WALL:
            estado = follow_wall();
            break;
        case Estado::IDLE:
            break;
    }
}

SpecificWorker::Estado SpecificWorker::turn(RoboCompLidar3D::TPoints points){
    static bool reset = true;
    bool right_left;
    if(reset){
        right_left = first_point.x > 0;
        reset=false;
    }

    auto first_point = points[points.size()/2];
    auto last_point = points[points.size()/2 + CENTRAL_POINTS_DIFF];

    int y_diff = last_point.y-first_point.y;
    int x_diff = last_point.x-first_point.x;


    double angle = std::abs(std::atan2(y_diff, x_diff));
    qInfo() << "ANGULO: " << angle;
    qInfo() << "First: X: " << first_point.x << "  Y: " << first_point.y;
    qInfo() << "Last: X: " << last_point.x << "  Y: " << last_point.y;
    qInfo() << "DIFF: X: " << x_diff << "  Y: " << y_diff;
    //if (angle > (std::numbers::pi/2) + THRESHOLD || angle < (std::numbers::pi/2) + -THRESHOLD) {
    if (std::abs(x_diff) > 10) {
        // STOP the robot && START
        omnirobot_proxy->setSpeedBase(0, 0,  right_left ? 0.5 : -0.5);
        return Estado::TURN;
    } else {
        //start the robot
        try {
            omnirobot_proxy->setSpeedBase(0, 0, 0.0);
            reset = true;
            return Estado::STRAIGHT_LINE;
        }
        catch (const Ice::Exception &e) {
            std::cout << "Error reading from Camera" << e << std::endl;
            return Estado::IDLE;
        }
    }
}

SpecificWorker::Estado SpecificWorker::straight_line(auto primer_elemento){
    const float MIN_DISTANCE = 1000;
        //start the robot
    try {
        if (std::hypot(primer_elemento.x, primer_elemento.y) < MIN_DISTANCE) {
            // STOP the robot && START
            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);
            return Estado::TURN;
        }
        else {
            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);
            return  Estado::STRAIGHT_LINE;
        }
    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return Estado::IDLE;
    }
}

Estado SpecificWorker::follow_wall(auto closest_wall_point, auto closest_forward_point)
{
    const float MIN_DISTANCE = 1000;
    const float THRESHOLD = 100;
    //start the robot
    try {
        if (std::hypot(closest_forward_point.x, closest_forward_point.y) < MIN_DISTANCE) {
            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);
            return Estado::TURN;
        } if (std::hypot(closest_wall_point.x, closest_wall_point.y) < MIN_DISTANCE-THRESHOLD) {
            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, -0.5);
            return  Estado::STRAIGHT_LINE;
        } if (std::hypot(closest_wall_point.x, closest_wall_point.y) > MIN_DISTANCE+THRESHOLD) {
            omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, +0.5);
            return  Estado::STRAIGHT_LINE;
        }
        omnirobot_proxy->setSpeedBase(1000 / 1000.f, 0, 0);
        return  Estado::STRAIGHT_LINE;

    }
    catch (const Ice::Exception &e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return Estado::IDLE;
    }
}



int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints& points, AbstractGraphicViewer *scene,
                                std::vector<RoboCompLidar3D::TPoint>& forward_points,
                                std::vector<RoboCompLidar3D::TPoint>& close_points) {
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

        color.setNamedColor(i/points.size() > 0.4 && i/points.size() < 0.6 ?"Green":"Red");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }

    for (const auto &p: forward_points) {
        QColor color("Orange");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }
    for (const auto &p: close_points) {
        QColor color("Cyan");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);

        i++;
    }

    auto p = points[close_points.size()/2];
    auto point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Magenta")),
                                       QBrush(QColor("Magenta")));
    point->setPos(p.x, p.y);
    borrar.push_back(point);

    try {
        p = close_points.at(close_points.size() / 2 + CENTRAL_POINTS_DIFF);
        point = viewer->scene.addRect(-50, -50, 100, 100,
                                      QPen(QColor("Yellow")),
                                      QBrush(QColor("Yellow")));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    } catch (const std::out_of_range &e) {
        
    }


}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
