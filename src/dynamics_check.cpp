#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QDebug"
#include "qmessagebox.h"


void HandGuidingForm::on_cBRobotType_currentTextChanged(const QString &arg1)
{
    double defaultPayLoad = 0;
    if(arg1 == "G3")
    {
        defaultPayLoad = 3.0;
    }
    else if(arg1 == "G6")
    {
        defaultPayLoad = 6.0;
    }
    else if(arg1 == "G12")
    {
        defaultPayLoad = 12.0;
    }
    else if(arg1 == "G18")
    {
        defaultPayLoad = 18.0;
    }

//    ui->lEPayLoad->setText(defaultPayLoad);
}


void HandGuidingForm::on_lEPayLoad_returnPressed()
{
    qDebug()<<ui->lEPayLoad->text();
}
