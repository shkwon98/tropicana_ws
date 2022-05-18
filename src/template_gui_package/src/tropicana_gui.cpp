#include "tropicana_gui.h"
#include "ui_tropicana_gui.h"

TROPICANA_GUI::TROPICANA_GUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::TROPICANA_GUI)
{
  ui->setupUi(this);
}

TROPICANA_GUI::~TROPICANA_GUI()
{
  delete ui;
}
