
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "../include/turtlegui/main_window.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace turtlegui {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(kill_ros()), this, SLOT(kill_all()));   
}

MainWindow::~MainWindow() {kill_all();}

void MainWindow::kill_all()
{
	system("pkill roslaunch");		//kills all roslaunch processes
	close();
}
void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::on_button_publish_path_clicked()	//handles publish_path button press
{
	if (traj_path.size() != 0)
	{
		//let qnode publish path
		qnode.button_publish_path_pressed(traj_path);		
		traj_path.clear();		//clear path
	}
}

void MainWindow::on_button_add_pose_clicked()	//handles add_pose button press
{
	std::string x = ui.qline_x->text().toStdString();
	std::string y = ui.qline_y->text().toStdString();
	q_traj_list.append(QString::fromStdString(x + "," + y));
	update_trajectory_path();
}
void MainWindow::on_button_clear_path_clicked()		//handles clear_path button press
{
	q_traj_list.clear();
	update_trajectory_path();
}
void MainWindow::on_button_browse_file_clicked()	//handles browse_file button press
{	
	//loads csv file
	QString s1 = QFileDialog::getOpenFileName(this, "Open a file", "directoryToOpen","CSV file (*.csv);; Excel (*.xml)");
	std::ifstream myFile(s1.toStdString());
	std::string s,x,y,z, qx, qy, qz;
	x = y = z = qx = qy = qz = "0.0";
	std::string qw = "1.0";
	q_traj_list.clear();

	//parses file
	while(myFile)
    {

		if (!std::getline( myFile, s )) break;

		std::istringstream ss( s );
    	std::vector <std::string> pose;
		while (ss)
		{
			std::string s;
			if (!std::getline( ss, s, ',' )) break;
			pose.push_back( s );
		}
		x = pose[0];
		y = pose[1];
		q_traj_list.append(QString::fromStdString(x + "," + y));
    }
	if (!myFile.eof())
  	{
    	std::cout << "err!\n";
  	}
	update_trajectory_path();
    myFile.close();
}
void MainWindow::on_button_launch_clicked()		//handles launch button press
{
	
	bool sim = ui.qradio_sim->isChecked();		//checks type of environment to launch
	if(sim)
	{
		QString command = "roslaunch turtlebot3_bringup turtlebot3_sim_nav_control.launch &";		//simulation 
		system(qPrintable(command));
	}
	else
	{
		QString command = "roslaunch turtlebot3_bringup turtlebot3_physical_nav_control.launch &";		//real world
		system(qPrintable(command));
	}
	
}
void MainWindow::on_button_advanced_controller_options_clicked()		//handles launch button press
{
	QString command = "rosrun rqt_gui rqt_gui &";		//launches rqt_gui (rqt_reconfigure)
	system(qPrintable(command));
}
void MainWindow::update_trajectory_path()	//updates current path of Turtlebot
{		
	

	//updates the trajectory list in the GUI
	QStringListModel *model;
	model = new QStringListModel(this);
	model->setStringList( QStringList{} );
	model->setStringList(q_traj_list);
	ui.list_path->setModel(model);

	//updates actual path
	std::string x,y;
	traj_path.clear();
	for ( const auto& s : q_traj_list)
	{
		std::string i = s.toStdString();
		std::size_t first_index = i.find(",");
		if (first_index!=std::string::npos)
		{
			x = i.substr(0,first_index);
			y = i.substr(first_index + 1, i.length());
			geometry_msgs::PoseWithCovarianceStamped msg;
			msg.pose.pose.position.x = std::stof(x);
			msg.pose.pose.position.y = std::stof(y);
			msg.pose.pose.position.z = 0.0;
			msg.pose.pose.orientation.x = 0.0;
			msg.pose.pose.orientation.y = 0.0;
			msg.pose.pose.orientation.z = 0.0;
			msg.pose.pose.orientation.w = 1.0;
			traj_path.push_back(msg);
		}
	}
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}

