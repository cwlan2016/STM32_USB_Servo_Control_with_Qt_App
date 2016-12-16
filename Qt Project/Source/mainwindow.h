#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void update_matrix();
    ~MainWindow();
private slots:
	void handleBytesWritten(qint64 bytes);
    void serialReceived();
    void replyAction();
    void on_actionConnectButtonAction_triggered();

private:
    Ui::MainWindow *ui;

    QGraphicsScene *scene;
    QGraphicsTextItem *text;
};

#endif // MAINWINDOW_H
