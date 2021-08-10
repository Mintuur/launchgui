/**
 * @file /include/launchgui/main_window.hpp
 *
 * @brief Qt based gui for launchgui.
 *
 * @date November 2010
 **/
#ifndef launchgui_MAIN_WINDOW_H
#define launchgui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace launchgui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void AutoDriving();
    void Door();
    void Obstacle();
    void Parking();
    void Stair();
    void Start();
    void MD();
    void JOY();

    void All_stop();

    void Off_auto();
    void Off_door();
    void Off_obstacle();
    void Off_parking();
    void Off_stair();
    void Off_MD();
    void Off_JOY();

    void updateState();
    void getReady();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        QPixmap m_lightimg[2];
        QPixmap m_readyimg[2];

};

}  // namespace launchgui

#endif // launchgui_MAIN_WINDOW_H
