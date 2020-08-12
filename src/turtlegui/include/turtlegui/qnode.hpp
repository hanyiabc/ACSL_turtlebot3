/**
 * @file /include/turtlegui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef turtlegui_QNODE_HPP_
#define turtlegui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <vector>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace turtlegui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	void button_publish_path_pressed(std::vector<geometry_msgs::PoseWithCovarianceStamped> path);
	void run();
	void init();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

Q_SIGNALS:
    void kill_ros();

private:
	int init_argc;
	char** init_argv;
	std::string master_url;
	std::string host_url;
	std::vector<geometry_msgs::PoseWithCovarianceStamped> pub_msg;
	ros::Publisher path_publisher;
	ros::Publisher clear_path_publisher;
};

}  // namespace turtlegui

#endif /* turtlegui_QNODE_HPP_ */
