#ifndef turtlegui_MAIN_WINDOW_H
#define turtlegui_MAIN_WINDOW_H

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace turtlegui {


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();


	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	void on_button_publish_path_clicked();
	void on_button_clear_path_clicked();
	void on_button_launch_clicked();
	void on_button_advanced_controller_options_clicked();
	void on_button_add_pose_clicked();
	void on_button_browse_file_clicked();
	void update_trajectory_path();
	void kill_all();

private:
	Ui::MainWindowDesign ui;
	bool follow_waypoint_init;
	QNode qnode;
	std::vector<geometry_msgs::PoseWithCovarianceStamped> traj_path;
	QStringList q_traj_list;
};

}  

#endif 
