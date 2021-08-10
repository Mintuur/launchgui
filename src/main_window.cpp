/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/launchgui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace launchgui {

int ros_topic_data;
bool ros_status_flag = 0;

extern int State[7];
extern int Ready;

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
        : QMainWindow(parent)
	, qnode(argc,argv)
{

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }

    QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(updateState()));
    QObject::connect(&qnode, SIGNAL(statusUpdated()), this, SLOT(getReady()));

    //QObject::connect(ui.Button_getmission, SIGNAL(clicked()), this, SLOT(launch_getmission()));
    //QObject::connect(ui.Button_Web, SIGNAL(clicked()), this, SLOT(Refresh_Web()));

    QObject::connect(ui.Button_AutoDriving, SIGNAL(clicked()), this, SLOT(AutoDriving()));
    QObject::connect(ui.Button_Door, SIGNAL(clicked()), this, SLOT(Door()));
    QObject::connect(ui.Button_Obstacle, SIGNAL(clicked()), this, SLOT(Obstacle()));
    QObject::connect(ui.Button_Parking, SIGNAL(clicked()), this, SLOT(Parking()));
    QObject::connect(ui.Button_Stair, SIGNAL(clicked()), this, SLOT(Stair()));

    QObject::connect(ui.Button_md, SIGNAL(clicked()), this, SLOT(MD()));
    QObject::connect(ui.Button_Joy, SIGNAL(clicked()), this, SLOT(JOY()));

    QObject::connect(ui.Button_Start, SIGNAL(clicked()), this, SLOT(Start()));    
    QObject::connect(ui.Button_All_stop, SIGNAL(clicked()), this, SLOT(All_stop()));

    QObject::connect(ui.Off_AutoDriving, SIGNAL(clicked()), this, SLOT(Off_auto()));
    QObject::connect(ui.Off_Door, SIGNAL(clicked()), this, SLOT(Off_door()));
    QObject::connect(ui.Off_Obstacle, SIGNAL(clicked()), this, SLOT(Off_obstacle()));
    QObject::connect(ui.Off_Parking, SIGNAL(clicked()), this, SLOT(Off_parking()));
    QObject::connect(ui.Off_Stair, SIGNAL(clicked()), this, SLOT(Off_stair()));

    QObject::connect(ui.Off_md, SIGNAL(clicked()), this, SLOT(Off_MD()));
    QObject::connect(ui.Off_Joy, SIGNAL(clicked()), this, SLOT(Off_JOY()));



    /*********************
    ** Label
    **********************/

   m_lightimg[0].load(":/images/led-off.png");
   m_lightimg[1].load(":/images/led-on.png");

   m_readyimg[0].load(":/images/switch2.jpg");
   m_readyimg[1].load(":/images/switch1.jpg");


}


MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateState() {
    if(State[0] == 1){
        ui.state_label_1->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_1->setPixmap(m_lightimg[0]);
    }

    if(State[1] == 1){
        ui.state_label_2->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_2->setPixmap(m_lightimg[0]);
    }

    if(State[2] == 1){
        ui.state_label_3->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_3->setPixmap(m_lightimg[0]);
    }

    if(State[3] == 1){
        ui.state_label_4->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_4->setPixmap(m_lightimg[0]);
    }

    if(State[4] == 1){
        ui.state_label_5->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_5->setPixmap(m_lightimg[0]);
    }

    if(State[5] == 1){
        ui.state_label_6->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_6->setPixmap(m_lightimg[0]);
    }

    if(State[6] == 1){
        ui.state_label_7->setPixmap(m_lightimg[1]);
    }
    else{
        ui.state_label_7->setPixmap(m_lightimg[0]);
    }

}

void MainWindow::getReady() {
    if(Ready == 1){
        ui.get_ready->setPixmap(m_readyimg[1]);
    }
    else{
        ui.get_ready->setPixmap(m_readyimg[0]);
    }
}


void MainWindow::AutoDriving() {
    ros_topic_data = 1;
    ros_status_flag = true;

    ui.mode_ready -> setText("Auto Driving");
}

void MainWindow::Door() {
    ros_topic_data = 2;
    ros_status_flag = true;

    ui.mode_ready -> setText("Open Door");
}

void MainWindow::Obstacle() {
    ros_topic_data = 3;
    ros_status_flag = true;

    ui.mode_ready -> setText("Obstacle");
}

void MainWindow::Parking() {
    ros_topic_data = 4;
    ros_status_flag = true;

    ui.mode_ready -> setText("Parking");
}

void MainWindow::Stair() {
    ros_topic_data = 5;
    ros_status_flag = true;

    ui.mode_ready -> setText("Stair");
}

void MainWindow::MD() {
    ros_topic_data = 60;
    ros_status_flag = true;
}

void MainWindow::JOY() {
    ros_topic_data = 70;
    ros_status_flag = true;
}

void MainWindow::Start() {
    ros_topic_data = 0;
    ros_status_flag = true;
}

void MainWindow::All_stop() {
    ros_topic_data = 9;
    ros_status_flag = true;
}

/*****************************************************************************
** Stop the selected thing
*****************************************************************************/

void MainWindow::Off_auto() {
    ros_topic_data = 11;
    ros_status_flag = true;
}

void MainWindow::Off_door() {
    ros_topic_data = 12;
    ros_status_flag = true;
}

void MainWindow::Off_obstacle() {
    ros_topic_data = 13;
    ros_status_flag = true;
}

void MainWindow::Off_parking() {
    ros_topic_data = 14;
    ros_status_flag = true;
}

void MainWindow::Off_stair() {
    ros_topic_data = 15;
    ros_status_flag = true;
}

void MainWindow::Off_MD() {     //md_driver
    ros_topic_data = 61;
    ros_status_flag = true;
}

void MainWindow::Off_JOY() {     //md_driver
    ros_topic_data = 71;
    ros_status_flag = true;
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "launchgui");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://10.30.94.232:11311/")).toString();
    QString host_url = settings.value("host_url", QString("10.30.93.206")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "launchgui");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace launchgui

