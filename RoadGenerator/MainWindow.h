#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui/QMainWindow>
#include "ui_MainWindow.h"
#include "ControlWidget.h"
#include "GLWidget.h"

class MainWindow : public QMainWindow {
Q_OBJECT

public:
	Ui::MainWindow ui;
	ControlWidget* controlWidget;
	GLWidget* glWidget;

public:
	MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0);
	~MainWindow();

protected:
	void keyPressEvent(QKeyEvent* e);
	void keyReleaseEvent(QKeyEvent* e);

public slots:
	void onNew();
	void onOpen();
	void onShowControlWidget();
};

#endif // MAINWINDOW_H
