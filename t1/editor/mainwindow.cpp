#include <QFileDialog>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Image.h"
#include "utils.h"

#include <iostream>

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent),ui(new Ui::MainWindow)
{
    _image = new Image();
    lbl_image = new QLabel();
    ui->setupUi(this);
    ui->scrollArea->setWidget(lbl_image);
    lbl_image->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    lbl_image->setAlignment(Qt::AlignCenter);
    //lbl_image->setScaledContents(true);
    setValuesLbls();
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::displayImage(QImage image)
{
    lbl_image->setPixmap(QPixmap::fromImage( image ));
    //lbl_image->adjustSize();
}

void MainWindow::setValuesLbls()
{
    ui->lbl_media->setText(QString::number( ui->sl_media->value() ));
    ui->lbl_median->setText(QString::number( ui->sl_median->value() ));
    ui->lbl_noise->setText(QString::number( ui->sl_noise->value() ));
    ui->lbl_rotation->setText(QString::number( ui->sl_rotation->value() ));
    ui->lbl_scale->setText(QString::number( ui->sl_scale->value() ));
}

void MainWindow::on_actionOpen_triggered()
{

    QString filename = QFileDialog::getOpenFileName(0,
                    tr("Open Image File"),
                    QString(),
                    "All files (*.*);; PNG (*.png);; JPG (*.jpg);; BITMAP (*.bmp)");
    _image = new Image(filename.toStdString());

    if(_image->have_image())
    {
        displayImage(_image->to_QImage());
    }
}
void MainWindow::on_actionGuardar_triggered()
{
    if(_image->have_image())
    {
        QString filename = QFileDialog::getSaveFileName(0,tr("Save Image File"),".png");
        cout<<"saved in: "<<filename.toStdString()<<endl;
        if(filename.size()>0)
        _image->Save(filename.toStdString());
    }

}

void MainWindow::on_bt_media_clicked()
{
    if(_image->have_image())
    {
        _image->filter_media(ui->sl_media->value());
        displayImage(_image->to_QImage());
        ui->sl_media->setValue(1);
    }

}

void MainWindow::on_bt_noise_clicked()
{
    if(_image->have_image())
    {
        float percent = (float)(ui->sl_noise->value())/100;
        _image->filter_noise(percent);
        displayImage(_image->to_QImage());
        ui->sl_noise->setValue(1);
    }
}

void MainWindow::on_bt_median_clicked()
{
    if(_image->have_image())
    {
        _image->filter_mediana(ui->sl_median->value());
        displayImage(_image->to_QImage());
        ui->sl_median->setValue(1);
    }
}

void MainWindow::on_bt_equalize_clicked()
{setValuesLbls();
    if(_image->have_image())
    {
        _image->equalize();
        displayImage(_image->to_QImage());
    }
}

void MainWindow::on_bt_scale_clicked()
{
    if(_image->have_image())
    {
        _image->resizeImag(ui->sl_scale->value());
        displayImage(_image->to_QImage());
        ui->sl_scale->setValue(100);
    }
}

void MainWindow::on_bt_rotation_clicked()
{
    if(_image->have_image())
    {
        _image->rotateImg(ui->sl_rotation->value());
        displayImage(_image->to_QImage());
        ui->sl_rotation->setValue(0);
    }
}

void MainWindow::on_bt_fft_clicked()
{
    if(_image->have_image())
    {
        _image->fft();
    }
}

void MainWindow::on_sl_media_actionTriggered(int action)
{
    setValuesLbls();
}

void MainWindow::on_sl_media_sliderReleased()
{
    setValuesLbls();
}



//no uso
void MainWindow::on_sl_media_sliderMoved(int position)
{
    //setValuesLbls();
}

void MainWindow::on_sl_media_sliderPressed()
{
   // setValuesLbls();
}

void MainWindow::on_sl_noise_actionTriggered(int action)
{
    setValuesLbls();
}

void MainWindow::on_sl_noise_sliderReleased()
{
    setValuesLbls();
}

void MainWindow::on_sl_median_actionTriggered(int action)
{
    setValuesLbls();
}

void MainWindow::on_sl_median_sliderReleased()
{
    setValuesLbls();
}

void MainWindow::on_sl_scale_actionTriggered(int action)
{
    setValuesLbls();
}

void MainWindow::on_sl_scale_sliderReleased()
{
    setValuesLbls();
}

void MainWindow::on_sl_rotation_actionTriggered(int action)
{
    setValuesLbls();
}

void MainWindow::on_sl_rotation_sliderReleased()
{
    setValuesLbls();
}


