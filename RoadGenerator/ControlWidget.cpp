#include "ControlWidget.h"
#include "MainWindow.h"
#include "GLWidget.h"
#include "GraphUtil.h"
#include "RoadSegmentationUtil.h"

ControlWidget::ControlWidget(MainWindow* mainWin) : QDockWidget("Control Widget", (QWidget*)mainWin) {
	this->mainWin = mainWin;

	// set up the UI
	/*
	ui.setupUi(this);
	ui.checkBoxRoadTypeAvenue->setChecked(true);
	ui.checkBoxRoadTypeLocalStreet->setChecked(true);
	ui.lineEditGridMaxIteration->setText("2");
	ui.lineEditNumBins->setText("9");
	ui.lineEditMinTotalLength->setText("3000");
	ui.lineEditMinMaxBinRatio->setText("0.5");
	ui.lineEditGridAngleThreshold->setText("0.1");
	ui.lineEditGridVotingThreshold->setText("0.7");
	ui.lineEditGridExtendingDistanceThreshold->setText("20");

	ui.lineEditRadialMaxIteration->setText("2");
	ui.lineEditScale1->setText("0.05");
	ui.lineEditScale2->setText("0.1");
	ui.lineEditCenterErrorTol2->setText("200");
	ui.lineEditAngleThreshold2->setText("0.4");
	ui.lineEditScale3->setText("0.2");
	ui.lineEditCenterErrorTol3->setText("200");
	ui.lineEditAngleThreshold3->setText("0.2");
	ui.lineEditRadialVotingThreshold->setText("0.7");
	ui.lineEditRadialSeedDistance->setText("200");
	ui.lineEditRadialExtendingAngleThreshold->setText("0.2");

	// register the event handlers
	connect(ui.pushButtonDetectGrid, SIGNAL(clicked()), this, SLOT(detectGrid()));
	connect(ui.pushButtonDetectPlaza, SIGNAL(clicked()), this, SLOT(detectPlaza()));
	connect(ui.pushButtonDetectRadial, SIGNAL(clicked()), this, SLOT(detectRadial()));
	connect(ui.pushButtonDetectGridRadial, SIGNAL(clicked()), this, SLOT(detectGridRadial()));
	*/

	hide();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Event handlers

