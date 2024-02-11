#include "voxel_filtering.h"
#include "ui_voxel_filtering.h"

voxel_filtering::voxel_filtering(QWidget *parent) : QDialog(parent), ui(new Ui::voxel_filtering) {
    ui->setupUi(this);
}

voxel_filtering::~voxel_filtering() {
    delete ui;
}

void voxel_filtering::on_buttonBox_accepted() {
    emit sendData(ui->lineEdit->text());

    this->close();
}
