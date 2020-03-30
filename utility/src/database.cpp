#include "../include/database.h"
#include <QDir>
//#include "common.h"
#include <limits>

#define MAX_SIZE (PATH_MAX+1)

DataBase::DataBase():IsOpen(false)
{
}

DataBase::~DataBase()
{
    if(IsOpen)
        db_.close();
}

bool DataBase::openDataBase(QString name)
{
    if(IsOpen)
        db_.close();
//    QString dbName = QString("%1/Database/%2.db").arg(Common::getCommonHandle()->m_pluginPath).arg(name);
//    QString dbName = "/root/HandGuidingExample/Database/" + name + ".db";

    QString dbName = "../../Database/" + name + ".db";

    if(QSqlDatabase::contains("qt_sql_default_connection"))
      db_ = QSqlDatabase::database("qt_sql_default_connection");
    else
      db_ = QSqlDatabase::addDatabase("QSQLITE", dbName);

    db_.setHostName("localhost");
    db_.setUserName("root");
    db_.setPassword("root");

    db_.setDatabaseName(dbName);
    qDebug()<<"Database path:"<<dbName;

    name_ = name;
    IsOpen = db_.open();
    if(!IsOpen)
        qDebug() <<"Open data base error!";
    return IsOpen;
}

bool DataBase::createTable(QString name)
{
    QSqlQuery query(db_);
    bool success = query.exec(name);
    if(!success)
    {
        QSqlError lastError = query.lastError();
        qDebug() << lastError.driverText() << QString(QObject::tr("creat table failed"));
        return false;
    }
    return true;
}

bool DataBase::queryValue(QString tableName, QString key, QString &value)
{
    QSqlQuery query(db_);
    bool success = query.exec(QString("SELECT * FROM %1").arg(tableName));
    while(query.next())
    {
       if(query.value(0).toString() == key)
       {
           value = query.value(1).toString();
           break;
       }
    }
    return success;
}

bool DataBase::queryValue(QString tableName, QString key, double *value)
{
    QSqlQuery query(db_);
    bool success = query.exec(QString("SELECT * FROM %1").arg(tableName));
    while(query.next())
    {
       if(query.value(0).toString() == key)
       {
           value[0] = query.value(1).toDouble();
           value[1] = query.value(2).toDouble();
           value[2] = query.value(3).toDouble();
           value[3] = query.value(4).toDouble();
           value[4] = query.value(5).toDouble();
           value[5] = query.value(6).toDouble();
           break;
       }
    }
    return success;
}

bool DataBase::updateByKey(QString tableName, QString key, QString value)
{
    QSqlQuery query(db_);
    QString str = QString("UPDATE %1 SET name = \'%2\' WHERE para = \'%3\'").arg(tableName).arg(value).arg(key);

    bool success = query.exec(str);
    if(!success)
    {
        QSqlError lastError = query.lastError();
        qDebug() << lastError.driverText() << QString(QObject::tr("update fail"));
    }
    return success;
}

bool DataBase::updateByID(QString tableName, QString key, QString value, int id)
{
    openDataBase(name_);            //add this to fix the close status by other user
    query_ = QSqlQuery(db_);

    //qDebug() <<db_.databaseName()<<  "QString(QObject::tr()"<< db_.connectionNames();
    QString name = "v" + QString::number(id);
    QString str = QString("UPDATE %1 SET %2 = \'%3\' WHERE para = \'%4\'").arg(tableName).arg(name).arg(value).arg(key);

    bool success = query_.exec(str);
    if(!success)
    {
        QSqlError lastError = query_.lastError();
        qDebug() << lastError.driverText() << QString(QObject::tr("update fail"));
    }
    return success;
}

bool DataBase::insert(QString tableName, QString key, QString value)
{
    bool success;
    QSqlQuery query(db_);
    query.prepare(QString("INSERT INTO %1 (para, name) VALUES (:para, :name)").arg(tableName)); //准备执行SQL查询
    query.bindValue(":para", key);   //
    query.bindValue(":name", value);
    success = query.exec();

//    QString str = QString(QObject::tr("insert into %1 values(%2, %3)")).arg(tableName).arg(key).arg(value);
//    success = query.exec(str);
    if(!success)
    {
        QSqlError lastError = query.lastError();
        qDebug() << lastError.driverText() << QString(QObject::tr("insert failed"));
        return false;
    }
    return true;
}

bool DataBase::insert(QString tableName, QString key, double value[])
{
    // default:  the size of value is 6
    QSqlQuery query(db_);
    bool success;
    query.prepare(QString("INSERT INTO %1 (para, v1, v2, v3, v4, v5, v6) VALUES (:para, :v1, :v2, :v3, :v4, :v5, :v6)").arg(tableName));
    query.bindValue(":para", key);
    query.bindValue(":v1", QString::number(value[0]));
    query.bindValue(":v2", QString::number(value[1]));
    query.bindValue(":v3", QString::number(value[2]));
    query.bindValue(":v4", QString::number(value[3]));
    query.bindValue(":v5", QString::number(value[4]));
    query.bindValue(":v6", QString::number(value[5]));

    success = query.exec();
    if(!success)
    {
        QSqlError lastError = query.lastError();
        qDebug() << lastError.driverText() << QString(QObject::tr("insert failed"));
        return false;
    }
    return true;
}
