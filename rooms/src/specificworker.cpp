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
#include <ranges>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>

#define DEGREE_TO_RADIAN(x) x * (M_PI / 180)
#define RADIAN_TO_DEGREE(x) x * (180 / M_PI)

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
    std::srand(std::time(nullptr));
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

RoboCompLidar3D::TPoints SpecificWorker::filterLidarPoints(const RoboCompLidar3D::TPoints& points) {
    const float Z_MAXHEIGHT = 2000;
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if( points, std::back_inserter(vectorPoints),
        [=](const auto& a) {return a.z < Z_MAXHEIGHT;});
    return vectorPoints;
}

RoboCompLidar3D::TPoints SpecificWorker::filterForwardPoints(const RoboCompLidar3D::TPoints& points,
        double ref_angle = M_PI / 2, double threshold = DEGREE_TO_RADIAN(6)) {
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if(points, std::back_inserter(vectorPoints),
        [=](const auto& a) {
            if (ref_angle ==  M_PI / 2 && a.y < 0) return false;
            auto angle = std::atan2(a.y, a.x);
            return std::abs(angle-ref_angle) < threshold;});
    return vectorPoints;
}

RoboCompLidar3D::TPoints SpecificWorker::filterClosePoints(const RoboCompLidar3D::TPoints& points) {
    RoboCompLidar3D::TPoints vectorPoints;
    std::ranges::copy_if( points, std::back_inserter(vectorPoints),
        [this](const auto& p) {return std::hypot(p.x,p.y,p.z) < MIN_DISTANCE + 600;});
    return vectorPoints;
}

RoboCompLidar3D::TPoint SpecificWorker::closestElement( const RoboCompLidar3D::TPoints& points) {
    return *std::ranges::min_element(points,
        [](const auto& a, const auto& b) {return std::hypot(a.x, a.y, a.z) < std::hypot(b.x, b.y, b.z);});
}

double SpecificWorker::calculateSpeed(double distance) const {
    constexpr auto slope = (MAX_FORWARD_SPEED - MIN_FORWARD_SPEED) / SLOW_DISTANCE;
    return distance > MIN_DISTANCE + SLOW_DISTANCE ? MAX_FORWARD_SPEED :
           slope * distance + MIN_FORWARD_SPEED - slope*MIN_DISTANCE;
}

double SpecificWorker::calculateRotationSpeed(double angle) const {
    constexpr auto slope = (MIN_ROTATION_SPEED - ROTATION_SPEED) / (M_PI / 2 - SLOW_ANGLE);
    return angle < SLOW_ANGLE ? ROTATION_SPEED :
        slope * angle + ROTATION_SPEED - slope * SLOW_ANGLE;
}

SpecificWorker::Lines SpecificWorker::filterLidarPointsFI(const RoboCompLidar3D::TPoints& points){
    Lines lines;

    constexpr float height[3] = {300,600,1000};

    auto angle = points[0].theta;
    const RoboCompLidar3D::TPoint * pointsHeights [3];
    for (const auto& p: points){
        if (p.theta != angle) {
            if (pointsHeights[0])
                lines.low.push_back(*pointsHeights[0]);
            if (pointsHeights[1])
                lines.middle.push_back(*pointsHeights[1]);
            if (pointsHeights[2])
                lines.high.push_back(*pointsHeights[2]);

            for (auto&i : pointsHeights)
                i= nullptr;
            angle = p.theta;
        }
        for (auto i : std::views::iota(0, 3)){
            if (p.z >= height[i] && (!pointsHeights[i] || p.z < pointsHeights[i]->z ) )
                pointsHeights[i] = &p;
        }
    }
    if (pointsHeights[0])
        lines.low.push_back(*pointsHeights[0]);
    if (pointsHeights[1])
        lines.middle.push_back(*pointsHeights[1]);
    if (pointsHeights[2])
        lines.high.push_back(*pointsHeights[2]);

    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_lines(const RoboCompLidar3D::TPoints &points) {
    Lines lines;
    const float LOW_LOW = 0;
    const float LOW_HIGH = 800;
    const float MIDDLE_LOW = 800;
    const float MIDDLE_HIGH = 1000;
    const float HIGH_LOW = 1000;
    const float HIGH_HIGH = 2500;

    for(const auto &p: points) {
        if(p.z > LOW_LOW and p.z < LOW_HIGH)
            lines.low.push_back(p);
        else if(p.z > MIDDLE_LOW and p.z < MIDDLE_HIGH)
            lines.middle.push_back(p);
        else if(p.z > HIGH_LOW and p.z < HIGH_HIGH)
            lines.high.push_back(p);
    }
    return lines;
}

SpecificWorker::Lines SpecificWorker::extract_peaks(const SpecificWorker::Lines &lines) {
    Lines peaks;

    constexpr float THRESHOLD = 1000; //mm

    for (const auto& both : iter::sliding_window(lines.low, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
            peaks.low.push_back(both[0].r < both[1].r ? both[0] : both[1]);
//            peaks.low.push_back(both[1]);
        }
    }
    for (const auto& both : iter::sliding_window(lines.middle, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
//            peaks.low.push_back(both[0]);
//            peaks.low.push_back(both[1]);
            peaks.middle.push_back(both[0].r < both[1].r ? both[0] : both[1]);
        }
    }
    for (const auto& both : iter::sliding_window(lines.high, 2)) {
        if (fabs(both[1].r - both[0].r) > THRESHOLD) {
//            peaks.low.push_back(both[0]);
//            peaks.low.push_back(both[1]);
            peaks.high.push_back(both[0].r < both[1].r ? both[0] : both[1]);
        }
    }
    return peaks;
}


void SpecificWorker::compute() {
    RoboCompLidar3D::TPoints ldata;
    try {
        ldata = lidar3d_proxy->getLidarData("helios", 0, 2 * M_PI, 1).points;
    }
    catch (const Ice::Exception& e) {
        std::cout << "Error reading from Camera" << e << std::endl;
        return;
    }

    auto vectorPoints = filterLidarPoints(ldata);
    if (vectorPoints.empty()) return;

    auto closePoints = filterClosePoints(vectorPoints);
    auto forwardPoints = filterForwardPoints(vectorPoints);

    auto three_heights = extract_lines(vectorPoints);
    auto lines = extract_peaks(three_heights);
    auto doors = get_doors(lines);

    draw_lidar(three_heights, viewer, forwardPoints, closePoints);
    draw_lines(lines, viewer);
    draw_doors(doors, viewer);

//    if (number_turns == 4) {
//        number_turns = 0;
//        MIN_DISTANCE += MIN_DISTANCE_STEP;
//        this->loops++;
//    }
    switch (estado) {
        case Estado::IDLE:
            break;
        case Estado::FIND_DOOR:
            estado = find_door(doors);
            break;
        case Estado::PASS_DOOR:
            estado = pass_door(doors);
            break;
        case Estado::GO_CENTER:
            estado = go_center(ldata);
            break;
        case Estado::CROSS_DOOR: {
            RoboCompLidar3D::TPoints backpoints;
            std::ranges::copy_if(ldata, std::back_inserter(backpoints),
                                 [=](const auto& a) {
                                     return a.y<800;});
            auto closest_point = closestElement(backpoints);
            estado = cross_door(closest_point);
            break;
        }
        case Estado::ALIGN:
            estado = align_door(doors);
            break;
    }

}

void SpecificWorker::move_robot(float advx, float advy, float rot)
{
    try
    {
        omnirobot_proxy->setSpeedBase(advx, advy, rot);
    }
    catch(const Ice::Exception &e){ std::cout << e << std::endl;}
}

void SpecificWorker::state_machine(const Doors &doors)
{
    switch (state)
    {
        case States::IDLE:
        {
            move_robot(0,0,0);
            break;
        }
        case States::SEARCH_DOOR:
        {
            //qInfo() << "SEARCH_DOOR";
            if(not doors.empty())
            {
//                door_target = doors[0];
//                move_robot(0,0,0);
//                state = States::GOTO_DOOR;
//                qInfo() << "First found";
                //door_target.print();
            }
            else
                move_robot(0,0,0.3);
            break;
        }
        case States::GOTO_DOOR:
        {
            //Info() << "GOTO_DOOR";
            //qInfo() << "distance " << door_target.dist_to_robot();
//            if(door_target.perp_dist_to_robot() < consts.DOOR_PROXIMITY_THRESHOLD)
//            {
//                move_robot(0,0,0);
//                qInfo() << "GOTO_DOOR Target achieved";
//                state = States::ALIGN;
//            }
//            else    // do what you have to do and stay in this state
//            {
//                float rot = -0.5 * door_target.perp_angle_to_robot();
//                float adv = consts.MAX_ADV_SPEED * break_adv(door_target.perp_dist_to_robot()) *
//                            break_rot(door_target.perp_angle_to_robot()) / 1000.f;
//                move_robot(0, adv, rot);
//            }
            break;
        }
        case States::ALIGN:
        {
//            if( fabs(door_target.angle_to_robot()) < 0.01)
//            {
//                move_robot(0,0,0);
//                state = States::GO_THROUGH;
//                return;
//            }
//            //qInfo() << door_target.angle_to_robot();
//            float rot = -0.4 * door_target.angle_to_robot();
//            move_robot(0,0,rot);
            break;
        }
        case States::GO_THROUGH:
        {
            move_robot(0,0,0);
            break;
        }
    }
}

SpecificWorker::Estado SpecificWorker::align_door(const Doors& doors){
    auto door_it = std::ranges::find(doors, target_door);
    target_door = *door_it;

    auto angle = target_door.get_angle();
    qInfo() << "ALIGN DOOR";
    qInfo() << RADIAN_TO_DEGREE(angle) <<"\n";


    if( fabs(angle- DEGREE_TO_RADIAN(90)) < 0.1) {
        move_robot(0,0,0);
        return Estado::CROSS_DOOR;
        }
    float rot = 0.4 * (angle-DEGREE_TO_RADIAN(90));
    move_robot(0,0,rot);
    return Estado::ALIGN;
}

SpecificWorker::Estado SpecificWorker::find_door(const Doors& doors){
    if (doors.size() == 0){
        omnirobot_proxy->setSpeedBase(1,0,0);
        return Estado::FIND_DOOR;
    }
    target_door = *std::ranges::min_element(doors.begin(), doors.end(), [](const Door& a, const Door& b){
        auto angle1 = std::abs(a.get_angle() - DEGREE_TO_RADIAN(90));
        auto angle2 = std::abs(b.get_angle() - DEGREE_TO_RADIAN(90));
        return angle1 < angle2;
    });
    return  Estado::PASS_DOOR;
}

SpecificWorker::Estado SpecificWorker::pass_door(const Doors& doors){

    auto door_it = std::ranges::find(doors, target_door);
    if (door_it == doors.end()){
        return Estado::FIND_DOOR;
    }

    target_door = *door_it;

    RoboCompLidar3D::TPoint vector{target_door.p2.x - target_door.p1.x, target_door.p2.y - target_door.p1.y};
    auto distance_vector = std::hypot(vector.x, vector.y);

    RoboCompLidar3D::TPoint perpendicular{-vector.y/distance_vector*1000, vector.x/distance_vector*1000};
    RoboCompLidar3D::TPoint p1 {target_door.middle.x+perpendicular.x, target_door.middle.y+perpendicular.y};
    RoboCompLidar3D::TPoint p2 {target_door.middle.x-perpendicular.x, target_door.middle.y-perpendicular.y};

    auto target = std::min(p1, p2, [](const auto& a, const auto& b){
        return std::hypot(a.x,a.y) < std::hypot(b.x,b.y);
    });

    double v_x = target.x;
    double v_y = target.y;
    double distance = std::hypot(target.x, target.y);

    double angle = std::atan2(target.y, target.x);
    double rel_angle = angle - M_PI/2;

    double THRESHOLD = DEGREE_TO_RADIAN(30);
    double rot_speed = std::abs(rel_angle) > THRESHOLD ? 1 : 1 / (THRESHOLD * THRESHOLD) * rel_angle * rel_angle;
    v_x = v_x/distance*MAX_FORWARD_SPEED * std::abs(rot_speed-1);
    v_y = v_y/distance*MAX_FORWARD_SPEED * std::abs(rot_speed-1);
    rot_speed = std::min(rot_speed*5, 1.0);

//    if (rel_angle < 0 ) rot_speed = -rot_speed;

    if (std::hypot(target.x, target.y) < 1000){
        return Estado::ALIGN;
    }
    float rot = 0.5 * rel_angle;

    qInfo() << "PASS DOOR\n"
       "\t Target door x: " << target.x <<"\n"
       "\t Target door y: " << target.y <<"\n"
       "\t Rot Speed: " << rot << "\n"
       "\t Angle: " << angle*180/M_PI << "\n";


//    float adv = consts.MAX_ADV_SPEED * break_adv(door_target.perp_dist_to_robot()) *
//            break_rot(door_target.perp_angle_to_robot()) / 1000.f;

    move_robot(v_y,  0, rot);
    return Estado::PASS_DOOR;
}
int SpecificWorker::startup_check() {
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

void SpecificWorker::draw_lidar(const Lines& points, AbstractGraphicViewer *scene,
                                const RoboCompLidar3D::TPoints& forward_points,
                                const RoboCompLidar3D::TPoints& close_points) {
    static std::vector<QGraphicsItem *> borrar;
    std::ranges::for_each(borrar,[this](auto& a) {viewer->scene.removeItem(a); delete a;});
    borrar.clear();

    for (const auto &p: points.high) {
        QColor color;

        color.setNamedColor("Red");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }

    for (const auto &p: forward_points) {
        QColor color("Orange");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }
    for (const auto &p: close_points) {
        QColor color("Cyan");
        auto point = viewer->scene.addRect(-25, -25, 50, 50,
                                           QPen(color),
                                           QBrush(color));
        point->setPos(p.x, p.y);
        borrar.push_back(point);
    }


    auto m_point = mean_point(points.low);
    auto point = viewer->scene.addRect(-50, -50, 100, 100,
                                       QPen(QColor("Green")),
                                       QBrush(QColor("Green")));
    point->setPos(m_point.x, m_point.y);
    borrar.push_back(point);
}


void SpecificWorker::draw_doors(const Doors &doors, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto& door : doors)
    {
        auto point =  viewer->scene.addLine(door.p1.x,door.p1.y,door.p2.x,door.p2.y,QPen(QColor("green"), 100));
        borrar.push_back(point);
    }
}

void SpecificWorker::draw_lines(const Lines& lines, AbstractGraphicViewer *viewer)
{
    static std::vector<QGraphicsItem*> borrar;
    for(auto &b : borrar) {
        viewer->scene.removeItem(b);
        delete b;
    }
    borrar.clear();

    for(const auto& line : lines.low) {
        auto point = viewer->scene.addRect(0,0, 100, 100,
                                           QPen("Black"),
                                           QBrush("Black"));
        point->setPos(line.x, line.y);
        borrar.push_back(point);
    }
//    for(const auto& line : lines.middle) {
//        auto point = viewer->scene.addRect(0,0, 100, 100,
//                                           QPen("Orange"),
//                                           QBrush("Orange"));
//        point->setPos(line.x, line.y);
//        borrar.push_back(point);
//    }
//    for(const auto& line : lines.high) {
//        auto point = viewer->scene.addRect(0,0, 100, 100,
//                                           QPen("Purple"),
//                                           QBrush("Purple"));
//        point->setPos(line.x, line.y);
//        borrar.push_back(point);
//    }
}

SpecificWorker::Doors SpecificWorker::get_doors(const Lines& lines){
    std::vector<Door> doors;
    constexpr float THRESHOLD = 100;
    auto dist = [](auto a, auto b) { return std::hypot(a.x-b.x, a.y-b.y); };
    auto near_door = [&doors, dist, THRESHOLD](auto d){
        for(auto &&old : doors) {
            if(dist(old.p1, d.p1) < THRESHOLD or
               dist(old.p2, d.p2) < THRESHOLD or
               dist(old.p1, d.p2) < THRESHOLD or
               dist(old.p2, d.p1) < THRESHOLD)
                return true;
            }
        return false;
    };

    for (auto && par: iter::combinations(lines.low, 2)){
        auto dist = std::hypot( par[1].x-par[0].x, par[1].y-par[0].y);
        if( dist < 1500 && dist > 500 ){
            auto door = Door{par[0],par[1]};
            if(not near_door(door) /*&& point_present(par[0], lines) && point_present(par[1], lines)*/)
                doors.push_back(door);
        }
    }
    return doors;
}

bool SpecificWorker::point_present(const RoboCompLidar3D::TPoint& point, const Lines& lines){
    auto distance_margin = [&point](const auto &x){return std::hypot(x.x-point.x, x.y-point.y) < 500;};
    return std::ranges::any_of(lines.middle,distance_margin) &&
           std::ranges::any_of(lines.high, distance_margin);
}

SpecificWorker::Estado SpecificWorker::cross_door(const RoboCompLidar3D::TPoint &point) {
//    double distance = std::hypot(point.x, point.y);
    static auto time = std::chrono::system_clock::now();
    static bool reset = true;

    if (reset){
        reset = false;
        time = std::chrono::system_clock::now() + 3s;
    }
    auto diff = time - std::chrono::system_clock::now();
    qInfo() << "CROSS DOOR";
//    qInfo() << "Diff:" << diff. << "\n";
    if (diff < 0s){
        reset = true;
        move_robot(0,0,0);
        return Estado::FIND_DOOR;
    }
    move_robot(3,0,0);
    return Estado::CROSS_DOOR;

//    "\t Distance: " << distance << "\n";
//    if (distance < 900) {
//        omnirobot_proxy->setSpeedBase(1, 0, 0);
//        return Estado::CROSS_DOOR;
//    }
//    return Estado::GO_CENTER;



}

SpecificWorker::Estado SpecificWorker::go_center(const RoboCompLidar3D::TPoints &points) {
    auto center = mean_point(points);
    auto distance = std::hypot(center.x, center.y);

    constexpr auto slope = (MAX_FORWARD_SPEED - MIN_FORWARD_SPEED) / SLOW_DISTANCE;

    auto speed = distance > (500 + SLOW_DISTANCE) ? MAX_FORWARD_SPEED :
                 slope * distance + MIN_FORWARD_SPEED - slope*500;

    auto vx = center.x*speed/distance;
    auto vy = center.y*speed/distance;

    qInfo() << "GO_CENTER\n"
               "\t Distance: " << distance <<
               "\t Speed: "<< speed << "\n";

    if (distance < 500) {
        omnirobot_proxy->setSpeedBase(1, 0, 0);
        return Estado::FIND_DOOR;
    }
    omnirobot_proxy->setSpeedBase(vy, -vx, 0);
    return Estado::GO_CENTER;
}

RoboCompLidar3D::TPoint SpecificWorker::mean_point(const RoboCompLidar3D::TPoints &points) {
    RoboCompLidar3D::TPoint point =  std::accumulate(points.begin(),points.end(), RoboCompLidar3D::TPoint{0,0},
        [&](const auto& a, const auto& b){
            auto distance = std::hypot(a.x, a.y);
            return RoboCompLidar3D::TPoint{a.x+b.x, a.y+b.y};
    });
    point.x /= points.size();
    point.y /= points.size();

    return point;
}



/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData
