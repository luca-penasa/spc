#ifndef SPLIT_POINT_CLOUD_DLG_H
#define SPLIT_POINT_CLOUD_DLG_H

#include <QDialog>

#include<ui_split_point_cloud_dlg.h>

class SplitPointCloudDlg : public QDialog,public Ui::SplitPointCloudDlg
{
    Q_OBJECT
    
public:
    explicit SplitPointCloudDlg(QWidget *parent = 0);

    

};

#endif // SPLIT_POINT_CLOUD_DLG_H
