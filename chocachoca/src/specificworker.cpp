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
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
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

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        viewer = new AbstractGraphicViewer(this, QRectF(-5000, -5000, 10000, 10000));
        viewer->add_robot(460,480,0,100,QColor("Blue"));
        viewer->show();
        viewer->activateWindow();
		timer.start(Period);

	}

}

void SpecificWorker::compute()
{
	try
	{
        auto ldata = lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1).points;

        qInfo()<<ldata.size();

        decltype(ldata) vectorPoints;

        std::copy_if(
                ldata.begin(),
                ldata.end(),
                std::back_inserter(vectorPoints),
                [](auto a){return a.z < 2000;});
        if (vectorPoints.empty()) return;



        draw_lidar(vectorPoints, viewer);

        ///control


        int offset = vectorPoints.size()/2 - vectorPoints.size()/5;
//        int offset = 0;

        auto primer_elemento = *std::min_element(vectorPoints.begin()+offset,
                  vectorPoints.end()-offset,
                  []( auto& a,  auto& b){
            return std::hypot(a.x,a.y,a.z) < std::hypot(b.x,b.y,b.z);});


        qInfo() << sqrt(primer_elemento.x*primer_elemento.x +
                        primer_elemento.y*primer_elemento.y +
                        primer_elemento.z*primer_elemento.z);
	}
	catch(const Ice::Exception &e)
	{
	  std::cout << "Error reading from Camera" << e << std::endl;
	}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::draw_lidar(RoboCompLidar3D::TPoints points, AbstractGraphicViewer* scene) {
    static std::vector<QGraphicsItem*> borrar;
//    std::for_each(borrar.begin(),borrar.end(),[this](auto a){viewer->scene.removeItem(a);});


    for (auto& p: borrar) {
        viewer->scene.removeItem(p);
        delete p;
    }


    borrar.clear();

    for(const auto& p: points){
        auto point  =viewer->scene.addRect(-50, -50, 100, 100, QPen(QColor("Green")), QBrush(QColor("Green")));
        point->setPos(p.x,p.y);
        borrar.push_back(point);
    }
}




/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

