#pragma execution_character_set("utf-8")

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    // 设置窗口标题和 logo
    this->setWindowTitle("CloudOne");
    this->setWindowIcon(QIcon(":/resourse/icon.ico"));

    m_publicIconMap[TREE_ITEM_ICON_DataItem] = QIcon(QStringLiteral(":/resourse/folder.png"));
    model = new QStandardItemModel(ui->treeView);
    model->setHorizontalHeaderLabels(QStringList()<<QStringLiteral("--cloud--DB-Tree--"));
    ui->treeView->setHeaderHidden(true);

    ui->treeView->setModel(model);
    ui->treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);  // 设置多选

    cloudptr.reset(new PointCloudT);
    cloud_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkNew<vtkGenericOpenGLRenderWindow> window;
    window->AddRenderer(cloud_viewer->getRendererCollection()->GetFirstRenderer());
    ui->openGLWidget->setRenderWindow(window.Get());
    cloud_viewer->setupInteractor(ui->openGLWidget->interactor(), ui->openGLWidget->renderWindow());
    ui->openGLWidget->update();

    // 创建菜单栏
    QMenuBar *menu_bar = new QMenuBar(this);
    this->setMenuBar(menu_bar);
    menu_bar->setStyleSheet("font-size : 16px");

    // 1、File 下拉列表
    QMenu *file_menu = new QMenu("File", menu_bar);
    QAction *open_action = new QAction("Open File");
    QAction *save_action = new QAction("Save File");
    QAction *exit_action = new QAction("Exit");
    // 添加动作到文件菜单
    file_menu->addAction(open_action);
    file_menu->addAction(save_action);
    file_menu->addSeparator();  // 添加菜单分隔符将 exit 单独隔离开
    file_menu->addAction(exit_action);
    // 把 File 添加到菜单栏
    menu_bar->addMenu(file_menu);

    // 2、Filter 下拉列表
    QMenu *filter_menu = new QMenu("Filter", menu_bar);
    QAction *voxel_action = new QAction("Voxel Filtering");
    filter_menu->addAction(voxel_action);
    menu_bar->addMenu(filter_menu);


    // 信号与槽函数链接
    connect(open_action, SIGNAL(triggered()), this, SLOT(open_clicked()));  // 打开文件
    connect(save_action, SIGNAL(triggered()), this, SLOT(save_clicked()));  // 保存文件
    connect(exit_action, SIGNAL(triggered()), this, SLOT(close()));         // 退出
    connect(voxel_action, SIGNAL(triggered()), this, SLOT(pressBtn_voxel()));
}

MainWindow::~MainWindow() {
    delete ui;
}

PointCloudT::Ptr MainWindow::pcl_voxel_filter(PointCloudT::Ptr cloud_in, float leaf_size) {
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(cloud_in);
    PointCloudT::Ptr cloud_out (new PointCloudT()) ;
    voxel_grid.filter(*cloud_out);

    return cloud_out;
}

void MainWindow::pressBtn_voxel() {
    dialog_voxel = new voxel_filtering();
    connect(dialog_voxel, SIGNAL(sendData(QString)), this, SLOT(voxel_clicked(QString)));

    if (dialog_voxel->exec() == QDialog::Accepted){}

    delete dialog_voxel;
}

// 体素采样
void MainWindow::voxel_clicked(QString data) {
    if (cloudptr->empty()) {
        QMessageBox::warning(this, "Warning", "None point cloud！");
        return;
    } else {
        if (data.isEmpty()) {
            QMessageBox::warning(this, "Warning", "Wrong format！");
            return;
        }

        float size = data.toFloat();
        auto cloud_out = pcl_voxel_filter(cloudptr, size);
        cloudptr = cloud_out;

        int size1 = static_cast<int>(cloudptr->size());
        QString PointSize = QString("%1").arg(size1);

        ui->textBrowser_2->clear();
        ui->textBrowser_2->insertPlainText("PCD number: " + PointSize);
        ui->textBrowser_2->setFont(QFont("Arial", 9, QFont::Normal));

        cloud_viewer->removeAllPointClouds();
        cloud_viewer->removeAllShapes();
        cloud_viewer->addPointCloud<pcl::PointXYZ>(cloudptr->makeShared(), std::to_string(cloud_vec.size()-1));
        cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, std::to_string(cloud_vec.size()-1));
        cloud_viewer->resetCamera();

        // 设置颜色处理器，将点云数据添加到 cloud_viewer 中
        const std::string axis = "z";
        pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloudptr, axis);
        cloud_viewer->addPointCloud(cloudptr, color_handler, "cloud");
        cloud_viewer->addPointCloud(cloudptr, "cloud");
    }
}

void MainWindow::open_clicked() {
    // this:代表当前对话框的父对象；tr("open file"):作为对话框的标题显示在标题栏中，使用 tr 函数表示这是一个需要翻译的文本
    // "":代表对话框的初始目录，这里为空表示没有指定初始目录
    // tr("pcb files(*.pcd *.ply *.txt) ;;All files (*.*)"):过滤器，决定在对话框中只能选择指定类型的文件
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "", tr("point cloud files(*.pcd *.ply) ;; All files (*.*)"));

    if (fileName.isEmpty()) {
        return;
    }
    if (fileName.endsWith("ply")) {
        qDebug() << fileName;
        if (pcl::io::loadPLYFile(fileName.toStdString(), *cloudptr) == -1) {
            qDebug() << "Couldn't read .ply file \n";
            return ;
        }
    } else if (fileName.endsWith("pcd")) {
        qDebug() << fileName;
        if (pcl::io::loadPCDFile(fileName.toStdString(), *cloudptr) == -1) {
            qDebug() << "Couldn't read .pcd file \n";
            return;
        }
    } else {
        QMessageBox::warning(this, "Warning", "Wrong format!");
    }

    cloud_vec.push_back(cloudptr->makeShared());
    cloud_index.push_back(1);

    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_folder")], QStringLiteral("cloud%1").arg(cloud_vec.size()-1));
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);  // 获取选中状态
    model->appendRow(itemFolder);

    int size = static_cast<int>(cloudptr->size());
    QString PointSize = QString("%1").arg(size);

    ui->textBrowser_2->clear();
    ui->textBrowser_2->insertPlainText("PCD number: " + PointSize);
    ui->textBrowser_2->setFont(QFont("Arial", 9, QFont::Normal));

    cloud_viewer->addPointCloud<pcl::PointXYZ>(cloudptr->makeShared(), std::to_string(cloud_vec.size()-1));
    // 设置点云大小
    cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, std::to_string(cloud_vec.size()-1));
    cloud_viewer->resetCamera();
    ui->openGLWidget->renderWindow()->Render();
    ui->openGLWidget->update();

    // 设置颜色处理器，将点云数据添加到 cloud_viewer 中
    const std::string axis = "z";
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloudptr, axis);
    cloud_viewer->addPointCloud(cloudptr, color_handler, "cloud");
    cloud_viewer->addPointCloud(cloudptr, "cloud");
}

void MainWindow::save_clicked() {
    int return_status;
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)"));

    if (cloudptr->empty()) {
        return;
    } else {
        if (filename.isEmpty()) {
            return;
        }
        if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloudptr);
        } else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
            return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
        } else {
            filename.append(".ply");
            return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr);
        }
        if (return_status != 0) {
            PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str());
            return;
        }
    }
}

void MainWindow::view_updata(std::vector<PointCloudT::Ptr> vector_cloud, std::vector<int> index) {
    cloud_viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkNew<vtkGenericOpenGLRenderWindow> window;
    window->AddRenderer(cloud_viewer->getRendererCollection()->GetFirstRenderer());
    ui->openGLWidget->setRenderWindow(window.Get());

    cloud_viewer->removeAllPointClouds();
    cloud_viewer->removeAllShapes();
    for (int i = 0; i<vector_cloud.size(); i++) {
        if (index[i] == 1) {
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>render(vector_cloud[i], "intensity");
            cloud_viewer->addPointCloud<pcl::PointXYZ>(vector_cloud[i], render, std::to_string(i));
            cloud_viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, std::to_string(i));
        }
    }
    cloud_viewer->resetCamera();
    ui->openGLWidget->update();
}

// 确定 index
void MainWindow::on_treeView_clicked(const QModelIndex &index) {
    QStandardItem* item = model->itemFromIndex(index);

    // 点云数量更改
    QStandardItemModel* model = static_cast<QStandardItemModel*>(ui->treeView->model());
    QModelIndex index_temp = ui->treeView->currentIndex();

    int size = static_cast<int>(cloud_vec[index_temp.row()]->size());
    QString PointSize = QString("%1").arg(size);

    ui->textBrowser_2->clear();
    ui->textBrowser_2->insertPlainText("Point cloud number: " + PointSize);
    ui->textBrowser_2->setFont(QFont("Arial", 9, QFont::Normal));

    // 可视化更改
    if (item == nullptr)
        return;
    if (item->isCheckable()) {
        //判断状态
        Qt::CheckState state = item->checkState();  // 获取当前的选择状态
        if (Qt::Checked == state) {
            cloud_index[index.row()] = 1;
        }
        if (Qt::Unchecked == state) {
             cloud_index[index.row()] = 0;
        }
        view_updata(cloud_vec, cloud_index);
    }
}
