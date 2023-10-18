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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

#define CENTRAL_POINTS_DIFF 400

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);



public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	bool startup_check_flag;
    AbstractGraphicViewer* viewer;

    enum class Estado {IDLE, FOLLOW_WALL, STRAIGHT_LINE, SPIRAL };
    Estado estado = Estado::STRAIGHT_LINE;


    Estado straight_line(auto primer_elemento);


    Estado follow_wall(RoboCompLidar3D::TPoints points);

    // filters points too high collected by Lidar
    std::vector<RoboCompLidar3D::TPoint> filterLidarPoints(const RoboCompLidar3D::TPoints &points);

    std::vector<RoboCompLidar3D::TPoint> filterForwardPoints(const std::vector<RoboCompLidar3D::TPoint>& points);

    RoboCompLidar3D::TPoint closestElement(const std::vector<RoboCompLidar3D::TPoint>::iterator& begin,
                                           const std::vector<RoboCompLidar3D::TPoint>::iterator& end);

    vector<RoboCompLidar3D::TPoint> filterClosePoints(const vector<RoboCompLidar3D::TPoint> &points);

    void draw_lidar(RoboCompLidar3D::TPoints &points, AbstractGraphicViewer *scene,
                    vector<RoboCompLidar3D::TPoint> &forward_points, vector<RoboCompLidar3D::TPoint> &close_points);
};

#endif
