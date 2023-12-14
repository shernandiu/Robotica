//
// Created by usuario on 28/11/23.
//

#ifndef ROOMS_DOOR_H
#define ROOMS_DOOR_H

struct Door {
    static constexpr double THRESHOLD = 400;
    RoboCompLidar3D::TPoint p1, p2, middle;
    Door(const RoboCompLidar3D::TPoint& p1, const RoboCompLidar3D::TPoint& p2): p1(p1), p2(p2){
        middle = RoboCompLidar3D::TPoint{(p1.x+p2.x)/2, (p1.y+p2.y)/2, (p1.z+p2.z)/2,
                                         (p1.intensity+p2.intensity)/2, (p1.phi+p2.phi)/2,
                                         (p1.theta+p2.theta)/2, (p1.r+p2.r)/2, (p1.distance2d+p2.distance2d)/2};
    }
    Door(const Door& other): p1(other.p1), p2(other.p2), middle(other.middle){};
    Door(){};
    bool operator==(const Door& other) const{
        return std::hypot(this->middle.x - other.middle.x , this->middle.y - other.middle.y ) < THRESHOLD;
    }
    double get_angle() const{
        return std::atan2(middle.y, middle.x);
    }
};



#endif //ROOMS_DOOR_H
