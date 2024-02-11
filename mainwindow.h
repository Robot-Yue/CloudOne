#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <voxel_filtering.h>
#include <vector>
#include <QMainWindow>
#include <QDebug>
#include <QColorDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTime>
#include <QDir>
#include <QFile>
#include <QtMath>
#include <QWindow>
#include <QAction>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QIcon>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStatusBar>
#include <QFont>
#include <QString>
#include <QTextBrowser>
#include <QDirIterator>
#include <QStandardItemModel>
#include <QModelIndex>

#include "QVTKOpenGLNativeWidget.h"
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include "vtkAutoInit.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#ifndef TREE_ITEM_ICON_DataItem
#define TREE_ITEM_ICON_DataItem QStringLiteral("treeItem_folder")
#endif

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer PCLViewer;
typedef std::shared_ptr<PointCloudT> PointCloudPtr;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    PointCloudT::Ptr pcl_voxel_filter(PointCloudT::Ptr cloud_in, float leaf_size);
    void view_updata(std::vector<PointCloudT::Ptr> vector_cloud, std::vector<int> index);

private slots:
    void open_clicked();  // 打开文件
    void save_clicked();  // 保存文件

    void on_treeView_clicked(const QModelIndex &index);

    // 点云滤波
    void pressBtn_voxel();
    void voxel_clicked(QString data);

private:
    Ui::MainWindow *ui;

    QMap<QString, QIcon> m_publicIconMap;   // 存放公共图标

    QStandardItemModel* model;
    QStandardItem* itemFolder;

    QModelIndex index_cloud;
    std::vector<PointCloudT::Ptr> cloud_vec;
    std::vector<int> cloud_index;

    // 点云名称
    std::vector<std::string> cloud_name{"0", "1", "2"};
    int point_size = 1;

    PointCloudPtr cloudptr;
    PCLViewer::Ptr cloud_viewer;

    voxel_filtering *dialog_voxel;
};
#endif // MAINWINDOW_H
