#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "Image.h"

#include <QMainWindow>
#include <QLabel>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionOpen_triggered();
    void displayImage(QImage image);

    void setValuesLbls();

    void on_bt_media_clicked();

    void on_bt_noise_clicked();

    void on_bt_median_clicked();

    void on_bt_equalize_clicked();

    void on_bt_scale_clicked();

    void on_bt_rotation_clicked();

    void on_bt_fft_clicked();


    void on_sl_media_actionTriggered(int action);

    void on_sl_media_sliderMoved(int position);

    void on_sl_media_sliderPressed();

    void on_sl_media_sliderReleased();

    void on_sl_noise_actionTriggered(int action);

    void on_sl_noise_sliderReleased();

    void on_sl_median_actionTriggered(int action);

    void on_sl_median_sliderReleased();

    void on_sl_scale_actionTriggered(int action);

    void on_sl_scale_sliderReleased();

    void on_sl_rotation_actionTriggered(int action);

    void on_sl_rotation_sliderReleased();

    void on_actionGuardar_triggered();

private:
    Ui::MainWindow *ui;
    Image* _image;
    QLabel* lbl_image;
};

#endif // MAINWINDOW_H
