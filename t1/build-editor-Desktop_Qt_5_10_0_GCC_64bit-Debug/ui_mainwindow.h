/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.10.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QAction *actionGuardar;
    QWidget *centralWidget;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QSlider *sl_media;
    QLabel *lbl_media;
    QPushButton *bt_media;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QSlider *sl_noise;
    QLabel *lbl_noise;
    QPushButton *bt_noise;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_3;
    QSlider *sl_median;
    QLabel *lbl_median;
    QPushButton *bt_median;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QPushButton *bt_equalize;
    QWidget *verticalLayoutWidget_5;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_4;
    QSlider *sl_scale;
    QLabel *lbl_scale;
    QPushButton *bt_scale;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_6;
    QSlider *sl_rotation;
    QLabel *lbl_rotation;
    QPushButton *bt_rotation;
    QWidget *verticalLayoutWidget_7;
    QVBoxLayout *verticalLayout_6;
    QPushButton *bt_fft;
    QMenuBar *menuBar;
    QMenu *menuFIle;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(914, 717);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        actionGuardar = new QAction(MainWindow);
        actionGuardar->setObjectName(QStringLiteral("actionGuardar"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        scrollArea = new QScrollArea(centralWidget);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setGeometry(QRect(180, 20, 721, 541));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 719, 539));
        scrollArea->setWidget(scrollAreaWidgetContents);
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(10, 20, 160, 94));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        sl_media = new QSlider(verticalLayoutWidget);
        sl_media->setObjectName(QStringLiteral("sl_media"));
        sl_media->setMinimum(1);
        sl_media->setMaximum(10);
        sl_media->setPageStep(1);
        sl_media->setSliderPosition(1);
        sl_media->setTracking(true);
        sl_media->setOrientation(Qt::Horizontal);
        sl_media->setInvertedControls(false);

        verticalLayout->addWidget(sl_media);

        lbl_media = new QLabel(verticalLayoutWidget);
        lbl_media->setObjectName(QStringLiteral("lbl_media"));
        lbl_media->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout->addWidget(lbl_media);

        bt_media = new QPushButton(verticalLayoutWidget);
        bt_media->setObjectName(QStringLiteral("bt_media"));

        verticalLayout->addWidget(bt_media);

        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 140, 160, 94));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(verticalLayoutWidget_2);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_2->addWidget(label_2);

        sl_noise = new QSlider(verticalLayoutWidget_2);
        sl_noise->setObjectName(QStringLiteral("sl_noise"));
        sl_noise->setMinimum(1);
        sl_noise->setMaximum(100);
        sl_noise->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(sl_noise);

        lbl_noise = new QLabel(verticalLayoutWidget_2);
        lbl_noise->setObjectName(QStringLiteral("lbl_noise"));
        lbl_noise->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_2->addWidget(lbl_noise);

        bt_noise = new QPushButton(verticalLayoutWidget_2);
        bt_noise->setObjectName(QStringLiteral("bt_noise"));

        verticalLayout_2->addWidget(bt_noise);

        verticalLayoutWidget_3 = new QWidget(centralWidget);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(10, 260, 160, 94));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(verticalLayoutWidget_3);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_3->addWidget(label_3);

        sl_median = new QSlider(verticalLayoutWidget_3);
        sl_median->setObjectName(QStringLiteral("sl_median"));
        sl_median->setMinimum(1);
        sl_median->setMaximum(10);
        sl_median->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(sl_median);

        lbl_median = new QLabel(verticalLayoutWidget_3);
        lbl_median->setObjectName(QStringLiteral("lbl_median"));
        lbl_median->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_3->addWidget(lbl_median);

        bt_median = new QPushButton(verticalLayoutWidget_3);
        bt_median->setObjectName(QStringLiteral("bt_median"));

        verticalLayout_3->addWidget(bt_median);

        verticalLayoutWidget_4 = new QWidget(centralWidget);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(10, 380, 160, 81));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        bt_equalize = new QPushButton(verticalLayoutWidget_4);
        bt_equalize->setObjectName(QStringLiteral("bt_equalize"));

        verticalLayout_4->addWidget(bt_equalize);

        verticalLayoutWidget_5 = new QWidget(centralWidget);
        verticalLayoutWidget_5->setObjectName(QStringLiteral("verticalLayoutWidget_5"));
        verticalLayoutWidget_5->setGeometry(QRect(410, 570, 241, 94));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_5);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(verticalLayoutWidget_5);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_5->addWidget(label_4);

        sl_scale = new QSlider(verticalLayoutWidget_5);
        sl_scale->setObjectName(QStringLiteral("sl_scale"));
        sl_scale->setMinimum(1);
        sl_scale->setMaximum(200);
        sl_scale->setValue(100);
        sl_scale->setOrientation(Qt::Horizontal);

        verticalLayout_5->addWidget(sl_scale);

        lbl_scale = new QLabel(verticalLayoutWidget_5);
        lbl_scale->setObjectName(QStringLiteral("lbl_scale"));
        lbl_scale->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_5->addWidget(lbl_scale);

        bt_scale = new QPushButton(verticalLayoutWidget_5);
        bt_scale->setObjectName(QStringLiteral("bt_scale"));

        verticalLayout_5->addWidget(bt_scale);

        verticalLayoutWidget_6 = new QWidget(centralWidget);
        verticalLayoutWidget_6->setObjectName(QStringLiteral("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(670, 570, 231, 94));
        verticalLayout_7 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        verticalLayout_7->setContentsMargins(0, 0, 0, 0);
        label_6 = new QLabel(verticalLayoutWidget_6);
        label_6->setObjectName(QStringLiteral("label_6"));

        verticalLayout_7->addWidget(label_6);

        sl_rotation = new QSlider(verticalLayoutWidget_6);
        sl_rotation->setObjectName(QStringLiteral("sl_rotation"));
        sl_rotation->setMinimum(-360);
        sl_rotation->setMaximum(360);
        sl_rotation->setOrientation(Qt::Horizontal);

        verticalLayout_7->addWidget(sl_rotation);

        lbl_rotation = new QLabel(verticalLayoutWidget_6);
        lbl_rotation->setObjectName(QStringLiteral("lbl_rotation"));
        lbl_rotation->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout_7->addWidget(lbl_rotation);

        bt_rotation = new QPushButton(verticalLayoutWidget_6);
        bt_rotation->setObjectName(QStringLiteral("bt_rotation"));

        verticalLayout_7->addWidget(bt_rotation);

        verticalLayoutWidget_7 = new QWidget(centralWidget);
        verticalLayoutWidget_7->setObjectName(QStringLiteral("verticalLayoutWidget_7"));
        verticalLayoutWidget_7->setGeometry(QRect(10, 490, 160, 71));
        verticalLayout_6 = new QVBoxLayout(verticalLayoutWidget_7);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        bt_fft = new QPushButton(verticalLayoutWidget_7);
        bt_fft->setObjectName(QStringLiteral("bt_fft"));

        verticalLayout_6->addWidget(bt_fft);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 914, 22));
        menuFIle = new QMenu(menuBar);
        menuFIle->setObjectName(QStringLiteral("menuFIle"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuFIle->menuAction());
        menuFIle->addAction(actionOpen);
        menuFIle->addAction(actionGuardar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Editor", nullptr));
        actionOpen->setText(QApplication::translate("MainWindow", "Abrir", nullptr));
#ifndef QT_NO_SHORTCUT
        actionOpen->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", nullptr));
#endif // QT_NO_SHORTCUT
        actionGuardar->setText(QApplication::translate("MainWindow", "Guardar", nullptr));
#ifndef QT_NO_SHORTCUT
        actionGuardar->setShortcut(QApplication::translate("MainWindow", "Ctrl+S", nullptr));
#endif // QT_NO_SHORTCUT
        label->setText(QApplication::translate("MainWindow", "difuminar (media)", nullptr));
        lbl_media->setText(QApplication::translate("MainWindow", "0", nullptr));
        bt_media->setText(QApplication::translate("MainWindow", "Aplicar", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Aplicar ruido (noise)", nullptr));
        lbl_noise->setText(QApplication::translate("MainWindow", "0", nullptr));
        bt_noise->setText(QApplication::translate("MainWindow", "Aplicar", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Limpiar ruido (median)", nullptr));
        lbl_median->setText(QApplication::translate("MainWindow", "0", nullptr));
        bt_median->setText(QApplication::translate("MainWindow", "Aplicar", nullptr));
        bt_equalize->setText(QApplication::translate("MainWindow", "Equalizar", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Escalar", nullptr));
        lbl_scale->setText(QApplication::translate("MainWindow", "0", nullptr));
        bt_scale->setText(QApplication::translate("MainWindow", "Aplicar", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Rotar", nullptr));
        lbl_rotation->setText(QApplication::translate("MainWindow", "0", nullptr));
        bt_rotation->setText(QApplication::translate("MainWindow", "Aplicar", nullptr));
        bt_fft->setText(QApplication::translate("MainWindow", "Ver magnitud FFT", nullptr));
        menuFIle->setTitle(QApplication::translate("MainWindow", "Archivo", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
