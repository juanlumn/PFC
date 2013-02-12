#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QFileDialog"

#include <QImage>
#include <QVector>
#include <QProcess>

#include <TCamaraKinect.h>
#include <QTime>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    OGLViewer = new TOpenGlViewer(640,480);
    OGLViewer->show();
    OGLViewer->SetVisualizationMode(1);
    connect(&timer,SIGNAL(timeout()),this,SLOT(NuevaImagen()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::NuevaImagen(){

    static QTime tiempo2;
    tiempo2.start();

    //If we get a new Image from the buffer
    if(CamaraKinect.ReadNewFrame()){
        //Read the buffer's image
        Inew=*CamaraKinect.GetImageBuffer();
        QImage ImagenO=Inew.GetQImage();
        int N=0,BestKey=0;
        TMatrizT Mmax,M,t;
        //For all the keyframes
        Inew.BestE(LK,BestKey,E);
        //Ransac method
        QImage imgfeat = Inew.ransac(LK[BestKey],M,N,E);
        Mmax=LM[BestKey]*M;

        //Vibration's filter
        static TMatrizT Mant;
        if(Mant.Diff1(Mmax)<0.15&&Mant.Diff2(Mmax)<0.20){
            Mmax = Mant;
        }else{
            Mant = Mmax;
        }

        TMatrizT tt;

        TMatrizT aux=Mmax;
        aux.Invert();
        //Camera's distance
        tt.ToTras(0,0,-1.45);
        t = aux*tt;
        OGLViewer->SetTransform(t);

        //Image representation. Features matched in green, without matched in red
        ui->label_4->setPixmap(QPixmap::fromImage(imgfeat));
        OGLViewer->SetImage(ImagenO);

        //Cases
        bool insertada=false;
        if(N<5){
        }else if(N<15){
            //If the program has 5 keyframes removes the oldest one and its matrix
            if (LK.count()==5){
                LK.removeFirst();
                LM.removeFirst();
            }
            //Appends the new keyframe and its matrix
            LK.append(Inew);
            LM.append(Mmax);
            ui->label_5->setPixmap(QPixmap::fromImage(LK[BestKey].GetQImage()));
            insertada=true;
        }else{
            ui->label_5->setPixmap(QPixmap::fromImage(LK[BestKey].GetQImage()));
        }

        //Combined image
        QImage Img2=QImage(ui->label_2->width(),ui->label_2->height(),QImage::Format_ARGB32);
        Img2.fill(Qt::green);
        float zoom = ui->horizontalSlider->value()/100.0;
        for(int i=0;i<LK.count();i++){
            CamaraKinect.DibujarImagen3D(Img2,LK[i],LM[i],zoom);
        }
        //Red frame
        Inew.ultimaimg();
        //Draws the new image
        if(!insertada) CamaraKinect.DibujarImagen3D(Img2,Inew,Mmax,zoom);
        ui->label_2->setPixmap(QPixmap::fromImage(Img2));

    }
    QString s;
    s.sprintf("NumKeyFrames = %d",LK.count());
    ui->label_9->setText(s);

    //Computes the FPS
    QString s1;
    float temp=tiempo2.elapsed();
    s1.sprintf("Tiempo = %2.0f milisegundos",temp);
    ui->label_3->setText(s1);
    tiempo2.restart();
    static int nframes = 0;
    static QTime tiempo;
    if(nframes==0) tiempo.start();
    nframes++;
    if(nframes==25){
        QString s;
        float fps = 1000*nframes/(float)tiempo.elapsed();
        s.sprintf("Fps = %2.2f",fps);
        ui->label->setText(s);
        nframes=0;
        tiempo.restart();
    }
    return;
}
//Start button
void MainWindow::on_pushButton_clicked()
{
    LK.clear();
    LM.clear();

    if(CamaraKinect.ReadNewFrame()){
        //First buffer's image will be the first reference keyframe
        Inew=*CamaraKinect.GetImageBuffer();
        //Appends the features from the reference's image
        LK.append(Inew);
        //Identity matrix, associated to the first keyframe
        TMatrizT M;
        LM.append(M);

        timer.start(0);
    }

}

void MainWindow::on_actionCargar_triggered()
{
    //Loads .OFF files
    c= QFileDialog::getOpenFileName(NULL,"Cargar Archivo .OFF",NULL,"*.off");
    if(c.isEmpty()==false){
        int vertices,caras,color;
        string tipo;
        ifstream fichero(c.toAscii(), ifstream::in);
        //Reads the type of file, number of vertex, faces and the color.
        fichero>>tipo;fichero>>vertices;fichero>>caras;fichero>>color;
        //Appends the vertex in the P vector
        P.clear();
        for (int i=0;i<vertices;i++){
            float aux;
            TPunto3D Paux;
            fichero>>aux;Paux.X()=aux;
            fichero>>aux;Paux.Y()=aux;
            fichero>>aux;Paux.Z()=aux;
            P.append(Paux);
            //If it's COFF gets the color in C
            if (tipo=="COFF"){
               int auxC;
               TColor Caux;
               fichero>>auxC;Caux.R()=auxC/255.0;
               fichero>>auxC;Caux.G()=auxC/255.0;
               fichero>>auxC;Caux.B()=auxC/255.0;
               fichero>>auxC;Caux.Alpha()=auxC/255.0;
               C.append(Caux);
            }
        }

        //Computes the limits in order to center the virtual object
        if(P.count()>0){
            TPunto3D min,max;
            min = max = P[0];
            for(int i=0;i<P.count();i++){
                TPunto3D p = P[i];
                if(p.X()<min.X()) min.X() = p.X();
                if(p.Y()<min.Y()) min.Y() = p.Y();
                if(p.Z()<min.Z()) min.Z() = p.Z();
                if(p.X()>max.X()) max.X() = p.X();
                if(p.Y()>max.Y()) max.Y() = p.Y();
                if(p.Z()>max.Z()) max.Z() = p.Z();
            }
            //Center
            TPunto3D centro = (min+max)/2;

            //The maximun size sets the scale
            TPunto3D tam = max - min;
            float tmax=tam.X();
            if(tam.Y()>tmax) tmax=tam.Y();
            if(tam.Z()>tmax) tmax=tam.Z();

            //Center, escalate and move the virtual image
            for(int i=0;i<P.count();i++){
                P[i] = (P[i]-centro)/tmax+TPunto3D(0,0,-1);
            }

        }

        //Appends the faces in F  
        F.clear();
        for (int i=0;i<caras;i++){
            int NumP,Punto;
            TCara Faux;
            NumP=0;
            fichero>>NumP;
            fichero>>Punto;Faux.F1()=Punto;
            fichero>>Punto;Faux.F2()=Punto;
            fichero>>Punto;Faux.F3()=Punto;
            if(NumP==4){
                fichero>>Punto;
                Faux.F4()=Punto;
            }
            F.append(Faux);
        }
        OGLViewer->SetScene(P,F,C);
    }
}
