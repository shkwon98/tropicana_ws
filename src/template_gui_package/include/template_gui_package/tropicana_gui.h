#ifndef TROPICANA_GUI_H
#define TROPICANA_GUI_H

#include <QMainWindow>
#include "ros/ros.h"

namespace Ui {
class TROPICANA_GUI;
}

class TROPICANA_GUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit TROPICANA_GUI(QWidget *parent = nullptr);
  ~TROPICANA_GUI();

private:
  Ui::TROPICANA_GUI *ui;
};

#endif // TROPICANA_GUI_H
