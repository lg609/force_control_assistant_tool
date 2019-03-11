#ifndef DATABASE_H
#define DATABASE_H

#include <QTextCodec>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QTime>
#include <QSqlError>
#include <QtDebug>
#include <QSqlDriver>
#include <QSqlRecord>
#include <QMap>

class DataBase
{
public:
    DataBase();
    ~DataBase();
    bool openDataBase(QString name);
    bool createTable(QString name);
    bool insert(QString tableName, QString key, QString value);            //insert
    bool insert(QString tableName, QString key, double value[]);
    bool queryValue(QString tableName, QString key, QString &value);
    bool queryValue(QString tableName, QString key, double *value);
    bool updateByKey(QString tableName, QString key, QString value);  //update
    bool updateByID(QString tableName, QString key, QString value, int id);

private:
    QSqlDatabase db_;
    bool IsOpen;
    QString name_;
};

#endif // DATABASE_H
