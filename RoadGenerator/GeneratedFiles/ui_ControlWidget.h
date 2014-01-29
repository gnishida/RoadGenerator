/********************************************************************************
** Form generated from reading UI file 'ControlWidget.ui'
**
** Created: Tue Jan 28 16:12:44 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLWIDGET_H
#define UI_CONTROLWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDockWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ControlWidget
{
public:
    QWidget *dockWidgetContents;
    QGroupBox *groupBox_3;
    QPushButton *pushButtonDetectGrid;
    QLabel *label_4;
    QLineEdit *lineEditNumBins;
    QLabel *label_5;
    QLineEdit *lineEditMinTotalLength;
    QLabel *label_6;
    QLineEdit *lineEditMinMaxBinRatio;
    QLabel *label_17;
    QLineEdit *lineEditGridVotingThreshold;
    QLabel *label_18;
    QLineEdit *lineEditGridMaxIteration;
    QLabel *label_20;
    QLineEdit *lineEditGridAngleThreshold;
    QLineEdit *lineEditGridExtendingDistanceThreshold;
    QLabel *label_21;
    QGroupBox *groupBox_4;
    QPushButton *pushButtonDetectPlaza;
    QGroupBox *groupBox_6;
    QPushButton *pushButtonDetectRadial;
    QLineEdit *lineEditScale1;
    QLabel *label_7;
    QLabel *label_8;
    QLineEdit *lineEditScale2;
    QLabel *label_9;
    QLineEdit *lineEditCenterErrorTol2;
    QLabel *label_10;
    QLineEdit *lineEditAngleThreshold2;
    QLineEdit *lineEditAngleThreshold3;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *lineEditCenterErrorTol3;
    QLineEdit *lineEditScale3;
    QLabel *label_14;
    QLineEdit *lineEditRadialVotingThreshold;
    QLineEdit *lineEditRadialSeedDistance;
    QLabel *label_15;
    QLineEdit *lineEditRadialExtendingAngleThreshold;
    QLabel *label_16;
    QLineEdit *lineEditRadialMaxIteration;
    QLabel *label_19;
    QGroupBox *groupBox_7;
    QPushButton *pushButtonDetectGridRadial;
    QCheckBox *checkBoxRoadTypeAvenue;
    QCheckBox *checkBoxRoadTypeLocalStreet;

    void setupUi(QDockWidget *ControlWidget)
    {
        if (ControlWidget->objectName().isEmpty())
            ControlWidget->setObjectName(QString::fromUtf8("ControlWidget"));
        ControlWidget->resize(218, 831);
        ControlWidget->setMinimumSize(QSize(218, 240));
        ControlWidget->setStyleSheet(QString::fromUtf8("background-color: rgb(181, 181, 181);"));
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        groupBox_3 = new QGroupBox(dockWidgetContents);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        groupBox_3->setGeometry(QRect(10, 60, 201, 211));
        pushButtonDetectGrid = new QPushButton(groupBox_3);
        pushButtonDetectGrid->setObjectName(QString::fromUtf8("pushButtonDetectGrid"));
        pushButtonDetectGrid->setGeometry(QRect(40, 170, 121, 31));
        label_4 = new QLabel(groupBox_3);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(10, 40, 46, 21));
        lineEditNumBins = new QLineEdit(groupBox_3);
        lineEditNumBins->setObjectName(QString::fromUtf8("lineEditNumBins"));
        lineEditNumBins->setGeometry(QRect(110, 40, 81, 20));
        lineEditNumBins->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_5 = new QLabel(groupBox_3);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(10, 60, 91, 21));
        lineEditMinTotalLength = new QLineEdit(groupBox_3);
        lineEditMinTotalLength->setObjectName(QString::fromUtf8("lineEditMinTotalLength"));
        lineEditMinTotalLength->setGeometry(QRect(110, 60, 81, 20));
        lineEditMinTotalLength->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(10, 80, 91, 21));
        lineEditMinMaxBinRatio = new QLineEdit(groupBox_3);
        lineEditMinMaxBinRatio->setObjectName(QString::fromUtf8("lineEditMinMaxBinRatio"));
        lineEditMinMaxBinRatio->setGeometry(QRect(110, 80, 81, 20));
        lineEditMinMaxBinRatio->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(10, 120, 101, 21));
        lineEditGridVotingThreshold = new QLineEdit(groupBox_3);
        lineEditGridVotingThreshold->setObjectName(QString::fromUtf8("lineEditGridVotingThreshold"));
        lineEditGridVotingThreshold->setGeometry(QRect(110, 120, 81, 20));
        lineEditGridVotingThreshold->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_18 = new QLabel(groupBox_3);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(10, 20, 71, 21));
        lineEditGridMaxIteration = new QLineEdit(groupBox_3);
        lineEditGridMaxIteration->setObjectName(QString::fromUtf8("lineEditGridMaxIteration"));
        lineEditGridMaxIteration->setGeometry(QRect(110, 20, 81, 20));
        lineEditGridMaxIteration->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_20 = new QLabel(groupBox_3);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(10, 100, 101, 21));
        lineEditGridAngleThreshold = new QLineEdit(groupBox_3);
        lineEditGridAngleThreshold->setObjectName(QString::fromUtf8("lineEditGridAngleThreshold"));
        lineEditGridAngleThreshold->setGeometry(QRect(110, 100, 81, 20));
        lineEditGridAngleThreshold->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        lineEditGridExtendingDistanceThreshold = new QLineEdit(groupBox_3);
        lineEditGridExtendingDistanceThreshold->setObjectName(QString::fromUtf8("lineEditGridExtendingDistanceThreshold"));
        lineEditGridExtendingDistanceThreshold->setGeometry(QRect(110, 140, 81, 20));
        lineEditGridExtendingDistanceThreshold->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_21 = new QLabel(groupBox_3);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(10, 140, 101, 21));
        groupBox_4 = new QGroupBox(dockWidgetContents);
        groupBox_4->setObjectName(QString::fromUtf8("groupBox_4"));
        groupBox_4->setGeometry(QRect(10, 280, 201, 61));
        pushButtonDetectPlaza = new QPushButton(groupBox_4);
        pushButtonDetectPlaza->setObjectName(QString::fromUtf8("pushButtonDetectPlaza"));
        pushButtonDetectPlaza->setGeometry(QRect(40, 20, 121, 31));
        groupBox_6 = new QGroupBox(dockWidgetContents);
        groupBox_6->setObjectName(QString::fromUtf8("groupBox_6"));
        groupBox_6->setGeometry(QRect(10, 350, 201, 291));
        pushButtonDetectRadial = new QPushButton(groupBox_6);
        pushButtonDetectRadial->setObjectName(QString::fromUtf8("pushButtonDetectRadial"));
        pushButtonDetectRadial->setGeometry(QRect(40, 250, 121, 31));
        lineEditScale1 = new QLineEdit(groupBox_6);
        lineEditScale1->setObjectName(QString::fromUtf8("lineEditScale1"));
        lineEditScale1->setGeometry(QRect(110, 40, 81, 20));
        lineEditScale1->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_7 = new QLabel(groupBox_6);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(10, 40, 46, 21));
        label_8 = new QLabel(groupBox_6);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(10, 60, 46, 21));
        lineEditScale2 = new QLineEdit(groupBox_6);
        lineEditScale2->setObjectName(QString::fromUtf8("lineEditScale2"));
        lineEditScale2->setGeometry(QRect(110, 60, 81, 20));
        lineEditScale2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_9 = new QLabel(groupBox_6);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(10, 80, 91, 21));
        lineEditCenterErrorTol2 = new QLineEdit(groupBox_6);
        lineEditCenterErrorTol2->setObjectName(QString::fromUtf8("lineEditCenterErrorTol2"));
        lineEditCenterErrorTol2->setGeometry(QRect(110, 80, 81, 20));
        lineEditCenterErrorTol2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_10 = new QLabel(groupBox_6);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(10, 100, 91, 21));
        lineEditAngleThreshold2 = new QLineEdit(groupBox_6);
        lineEditAngleThreshold2->setObjectName(QString::fromUtf8("lineEditAngleThreshold2"));
        lineEditAngleThreshold2->setGeometry(QRect(110, 100, 81, 20));
        lineEditAngleThreshold2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        lineEditAngleThreshold3 = new QLineEdit(groupBox_6);
        lineEditAngleThreshold3->setObjectName(QString::fromUtf8("lineEditAngleThreshold3"));
        lineEditAngleThreshold3->setGeometry(QRect(110, 160, 81, 20));
        lineEditAngleThreshold3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_11 = new QLabel(groupBox_6);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(10, 160, 91, 21));
        label_12 = new QLabel(groupBox_6);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(10, 140, 91, 21));
        label_13 = new QLabel(groupBox_6);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(10, 120, 46, 21));
        lineEditCenterErrorTol3 = new QLineEdit(groupBox_6);
        lineEditCenterErrorTol3->setObjectName(QString::fromUtf8("lineEditCenterErrorTol3"));
        lineEditCenterErrorTol3->setGeometry(QRect(110, 140, 81, 20));
        lineEditCenterErrorTol3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        lineEditScale3 = new QLineEdit(groupBox_6);
        lineEditScale3->setObjectName(QString::fromUtf8("lineEditScale3"));
        lineEditScale3->setGeometry(QRect(110, 120, 81, 20));
        lineEditScale3->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_14 = new QLabel(groupBox_6);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(10, 180, 101, 21));
        lineEditRadialVotingThreshold = new QLineEdit(groupBox_6);
        lineEditRadialVotingThreshold->setObjectName(QString::fromUtf8("lineEditRadialVotingThreshold"));
        lineEditRadialVotingThreshold->setGeometry(QRect(110, 180, 81, 20));
        lineEditRadialVotingThreshold->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        lineEditRadialSeedDistance = new QLineEdit(groupBox_6);
        lineEditRadialSeedDistance->setObjectName(QString::fromUtf8("lineEditRadialSeedDistance"));
        lineEditRadialSeedDistance->setGeometry(QRect(110, 200, 81, 20));
        lineEditRadialSeedDistance->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_15 = new QLabel(groupBox_6);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(10, 200, 101, 21));
        lineEditRadialExtendingAngleThreshold = new QLineEdit(groupBox_6);
        lineEditRadialExtendingAngleThreshold->setObjectName(QString::fromUtf8("lineEditRadialExtendingAngleThreshold"));
        lineEditRadialExtendingAngleThreshold->setGeometry(QRect(110, 220, 81, 20));
        lineEditRadialExtendingAngleThreshold->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_16 = new QLabel(groupBox_6);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(10, 220, 101, 21));
        lineEditRadialMaxIteration = new QLineEdit(groupBox_6);
        lineEditRadialMaxIteration->setObjectName(QString::fromUtf8("lineEditRadialMaxIteration"));
        lineEditRadialMaxIteration->setGeometry(QRect(110, 20, 81, 20));
        lineEditRadialMaxIteration->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_19 = new QLabel(groupBox_6);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(10, 20, 81, 21));
        groupBox_7 = new QGroupBox(dockWidgetContents);
        groupBox_7->setObjectName(QString::fromUtf8("groupBox_7"));
        groupBox_7->setGeometry(QRect(10, 650, 201, 61));
        pushButtonDetectGridRadial = new QPushButton(groupBox_7);
        pushButtonDetectGridRadial->setObjectName(QString::fromUtf8("pushButtonDetectGridRadial"));
        pushButtonDetectGridRadial->setGeometry(QRect(40, 20, 121, 31));
        checkBoxRoadTypeAvenue = new QCheckBox(dockWidgetContents);
        checkBoxRoadTypeAvenue->setObjectName(QString::fromUtf8("checkBoxRoadTypeAvenue"));
        checkBoxRoadTypeAvenue->setGeometry(QRect(20, 10, 70, 17));
        checkBoxRoadTypeLocalStreet = new QCheckBox(dockWidgetContents);
        checkBoxRoadTypeLocalStreet->setObjectName(QString::fromUtf8("checkBoxRoadTypeLocalStreet"));
        checkBoxRoadTypeLocalStreet->setGeometry(QRect(100, 10, 91, 17));
        ControlWidget->setWidget(dockWidgetContents);

        retranslateUi(ControlWidget);

        QMetaObject::connectSlotsByName(ControlWidget);
    } // setupUi

    void retranslateUi(QDockWidget *ControlWidget)
    {
        groupBox_3->setTitle(QApplication::translate("ControlWidget", "Grid Detection", 0, QApplication::UnicodeUTF8));
        pushButtonDetectGrid->setText(QApplication::translate("ControlWidget", "Detect Grid", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("ControlWidget", "Num Bins", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("ControlWidget", "Min Total Length", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("ControlWidget", "Min Max Bin Ratio", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("ControlWidget", "Vote Ratio Threshold", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("ControlWidget", "Max Iteration", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("ControlWidget", "Angle Threshold", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("ControlWidget", "Ext Dist Threshold", 0, QApplication::UnicodeUTF8));
        groupBox_4->setTitle(QApplication::translate("ControlWidget", "Plaza Detection", 0, QApplication::UnicodeUTF8));
        pushButtonDetectPlaza->setText(QApplication::translate("ControlWidget", "Detect Plaza", 0, QApplication::UnicodeUTF8));
        groupBox_6->setTitle(QApplication::translate("ControlWidget", "Radial Detection", 0, QApplication::UnicodeUTF8));
        pushButtonDetectRadial->setText(QApplication::translate("ControlWidget", "Detect Radial", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("ControlWidget", "Scale 1", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("ControlWidget", "Scale 2", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("ControlWidget", "Center Error Tol 2", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("ControlWidget", "Angle Threshold 2", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("ControlWidget", "Angle Threshold 3", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("ControlWidget", "Center Error Tol 3", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("ControlWidget", "Scale 3", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("ControlWidget", "Vote Ratio Threshold", 0, QApplication::UnicodeUTF8));
        lineEditRadialSeedDistance->setText(QString());
        label_15->setText(QApplication::translate("ControlWidget", "Seed Distance", 0, QApplication::UnicodeUTF8));
        lineEditRadialExtendingAngleThreshold->setText(QString());
        label_16->setText(QApplication::translate("ControlWidget", "Ext Angle Threshold", 0, QApplication::UnicodeUTF8));
        lineEditRadialMaxIteration->setText(QString());
        label_19->setText(QApplication::translate("ControlWidget", "Max Iteration", 0, QApplication::UnicodeUTF8));
        groupBox_7->setTitle(QApplication::translate("ControlWidget", "Grid / Radial Detection", 0, QApplication::UnicodeUTF8));
        pushButtonDetectGridRadial->setText(QApplication::translate("ControlWidget", "Detect Grid / Radial", 0, QApplication::UnicodeUTF8));
        checkBoxRoadTypeAvenue->setText(QApplication::translate("ControlWidget", "Avenues", 0, QApplication::UnicodeUTF8));
        checkBoxRoadTypeLocalStreet->setText(QApplication::translate("ControlWidget", "Local Streets", 0, QApplication::UnicodeUTF8));
        Q_UNUSED(ControlWidget);
    } // retranslateUi

};

namespace Ui {
    class ControlWidget: public Ui_ControlWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLWIDGET_H
