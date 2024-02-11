#ifndef VOXEL_FILTERING_H
#define VOXEL_FILTERING_H

#include <QDialog>
#include <QString>

namespace Ui {
class voxel_filtering;
}

class voxel_filtering : public QDialog {
    Q_OBJECT

signals:
    void sendData(QString data);

public:
    explicit voxel_filtering(QWidget *parent = nullptr);
    ~voxel_filtering();

private slots:
    void on_buttonBox_accepted();

private:
    Ui::voxel_filtering *ui;
};

#endif // VOXEL_FILTERING_H
