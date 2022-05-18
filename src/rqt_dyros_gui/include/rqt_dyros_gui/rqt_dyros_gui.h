#ifndef RQT_DYROS_H
#define RQT_DYROS_H

#include <rqt_gui_cpp/plugin.h>
#include <rqt_dyros_gui/ui_rqt_dyros_gui.h>
#include <QWidget>
#include <std_msgs/Float32.h>
#include <iostream>
#include <ros/ros.h>

namespace rqt_dyros_gui {

class RqtDyrosPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  RqtDyrosPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  ros::Publisher myPub;
  std_msgs::Float32 myMsg;
  ros::Subscriber mySub;
  void subCallback(const std_msgs::Float32ConstPtr &msg);

protected slots:
  virtual void pushbutton();

signals:
  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::RqtDyrosGuiWidget ui_;
  QWidget* widget_;
  ros::NodeHandle nh_;
};
} // namespace

#endif // my_namespace__my_plugin_H
