#ifndef TOPENGLVIEWER_H
#define TOPENGLVIEWER_H

#include <QtOpenGL/QGLWidget>
#include <QColor>
#include <QColorDialog>
#include <QMouseEvent>
#include <QDebug>
#include <transformacion/TMatrizT.h>
#include <QVector>
#include <TCamaraKinect.h>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string.h>

class TOpenGlViewer:public QGLWidget{
    Q_OBJECT
private:
    int vis_mode; //0 wireframe 1 solid
    QImage imagen;
    TMatrizT T;
    int W,H;
    QVector <TPunto3D> P;
    QVector <TCara> F;
    QVector <TColor> C;
    QVector<TPixel3D> puntos;
    QVector <TPunto3D> NM;
public:
   TOpenGlViewer(int w, int h, QWidget *parent = 0):QGLWidget(parent){
        W=w;
        H=h;
        vis_mode=1;
        T.ToIdent();
        return;
    }

    void SetScene(QVector <TPunto3D> p,QVector <TCara> f,QVector <TColor> c){
        QVector <TPunto3D> Normal;
        P=p;
        F=f;
        C=c;
        //Computes the normal for every faces
        for (int i=0;i<F.count();i++){
            TPunto3D p1 = P[F[i].F1()];
            TPunto3D p2 = P[F[i].F2()];
            TPunto3D p3 = P[F[i].F3()];

            TPunto3D v1 = p2-p1;v1.Normalizar();
            TPunto3D v2 = p3-p1;v2.Normalizar();

            TPunto3D n = v1*v2;n.Normalizar();
            //Stores the normals in Normal
            Normal.append(n);
        }
        //Computes the normal for every vertex
        for (int x=0;x<P.count();x++){
            TPunto3D Nmed;
            int contador=0;
            for (int i=0;i<F.count();i++){
                if(F[i].F1()==x||F[i].F2()==x||F[i].F3()==x||F[i].F4()==x){
                    Nmed=Normal[i]+Nmed;
                    contador++;
                }
            }
            Nmed=Nmed/contador;Nmed.Normalizar();
            NM.append(Nmed);
        }
        return;
    }
    //Visualization settings
    void SetVisualizationMode(int mode){
        vis_mode = mode;
        updateGL();
        return;
    }

    void initializeGL(){
        glClearColor(0.0,0.0,0.0,1.0);
        resizeGL(W,H);
        gluLookAt(0,0,0,0,0,30,0,1,0);//Origin (0,0,0),Camera (0,0,30),"View up" vector (0,1,0)
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);
        glEnable(GL_COLOR_MATERIAL);
    }

    void resizeGL(int w, int h){
        //Sets width and height of the window
        setMinimumSize(W,H);
        setMaximumSize(W,H);
        glViewport(0,0,w,h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        //Kinect's opening angle is 58 degrees in horizontal and 45 in vertical
        //In "gluPerspective" we have to set the vertical opening angle
        if ( H==0) gluPerspective(45, (GLdouble)W,0.01, 20000.0);
        else gluPerspective(45, (GLdouble)W/(GLdouble)H,0.01, 20000.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        return;
    }

    void DrawCube(QVector <TPunto3D> P,QVector <TCara> F,QVector <TColor> C){
        QVector <TPunto3D> Paux=P;
        QVector <TPunto3D> NMaux=NM;
        //Transforms all pointss
        TMatrizT T1=T;
        T1.AnularT();
        for(int i=0;i<Paux.count();i++) {
            T.Transformar(Paux[i]);
            T1.Transformar(NMaux[i]);NMaux[i].Normalizar();
        }

        //Visualization mode
        switch(vis_mode){
        case 0: glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); break;
        case 1: glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); break;
        }
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);
        glEnable(GL_COLOR_MATERIAL);
        //Lights
        GLfloat Position[]={1.0f,1.0f,0.0f,1.0f};
        GLfloat Ambient[] = {1.0f, 1.0f, 1.0f, 1.0f};
        GLfloat Diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        GLfloat Specular[]= {1.0f, 1.0f, 1.0f, 1.0f};
        glLightfv(GL_LIGHT0, GL_POSITION, Position);
        glLightfv( GL_LIGHT0, GL_AMBIENT, Ambient);
        glLightfv( GL_LIGHT0, GL_DIFFUSE, Diffuse);
        glLightfv( GL_LIGHT0, GL_SPECULAR,Specular);
        glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 1.0);
        //Reflected light and brightness coefficient
        GLfloat mat_specular[] = { 1.0, 1.0, 1.0, 1.0 };
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        GLfloat mat_shininess[] = { 100.0 };
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
       
        //Draw the faces
        for (int i=0;i<F.count();i++){
            int numP=F[i].F1();
            TPunto3D n=NMaux[numP];
            TPunto3D p=Paux[numP];
            TColor c=C[numP];

            glBegin(GL_POLYGON);

            glColor4f(c.R(),c.G(),c.B(),c.Alpha());
            glNormal3f(n.X(),n.Y(),n.Z());
            glVertex3f(p.X(), p.Y(),p.Z());

            numP=F[i].F2();
            c=C[numP];
            glColor4f(c.R(),c.G(),c.B(),c.Alpha());
            n=NMaux[numP];
            glNormal3f(n.X(),n.Y(),n.Z());
            p=Paux[numP];
            glVertex3f(p.X(), p.Y(),p.Z());

            numP=F[i].F3();
            c=C[numP];
            glColor4f(c.R(),c.G(),c.B(),c.Alpha());
            n=NMaux[numP];
            glNormal3f(n.X(),n.Y(),n.Z());
            p=Paux[numP];
            glVertex3f(p.X(), p.Y(),p.Z());

            if(F[i].F4()!=-1){
                numP=F[i].F4();
                c=C[numP];
                glColor4f(c.R(),c.G(),c.B(),c.Alpha());
                n=NMaux[numP];
                glNormal3f(n.X(),n.Y(),n.Z());
                p=Paux[numP];
                glVertex3f(p.X(), p.Y(),p.Z());
            }
            glEnd();
        }

    }

    void DrawImagePoints(void){
        glEnable(GL_DEPTH_TEST);
        glBegin(GL_POINTS);
        for(int i=0;i<puntos.count();i++){
            TPixel3D p = puntos[i];
            glColor4f(p.r/256.0,p.g/256.0,p.b/256.0,0.5);
            glVertex3f(p.x,p.y,p.z);
        }
        glEnd();
        return;
    }

    void SetImagePoints(QVector<TPixel3D> l){
        puntos = l;
    }

    void SetImage(QImage img){
        imagen = img;
        resizeGL(img.width(),img.height());
        updateGL();
    }

    void SetTransform(TMatrizT t){
        T=t;
        updateGL();
        return;
    }

    void paintGL(){
        //Background Color
        glClearColor(1.0,1.0,1.0,1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //Backgrond Image
        glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
        QPainter painter(this);
        painter.drawImage(0,0,imagen);
        painter.end();

        //Antialiasing
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
        glLineWidth(1);

        glDisable(GL_LIGHTING);
        glEnable(GL_DOUBLEBUFFER);

        DrawCube(P,F,C);
        DrawImagePoints();

        return;
    }
};

#endif
