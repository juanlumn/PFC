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
    int vis_mode; //0 Wireframe 1 Solid
    int W,H;
    bool modelight;
    QImage imagen;
    QVector <TPunto3D> P;
    QVector <TCara> C;
    QVector<TPixel3D> puntos;
    TMatrizT T;
public:
    TOpenGlViewer(int w, int h, QWidget *parent = 0):QGLWidget(parent){
        W=w;
        H=h;
        vis_mode=1;
        T.ToIdent();
        return;
    }

    void SetScene(QVector <TPunto3D> p,QVector <TCara> c){
        P=p;
        C=c;
        return;
    }

    //Visualization settings
    void SetVisualizationMode(int mode){
        vis_mode = mode;
        updateGL();
        return;
    }

    //Light settings
    void SetLightMode(bool mode){
        modelight=mode;
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


    void DrawCube(QVector <TPunto3D> P,QVector <TCara> C){
        QVector <TPunto3D> Paux=P;
        //Transforms all pointss
        for(int i=0;i<Paux.count();i++) T.Transformar(Paux[i]);

        //Visualization mode
        switch(vis_mode){
        case 0: glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); break;
        case 1: glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); break;
        }
        //      glColor3ub((GLubyte)100,(GLubyte)125,(GLubyte)100);
        glClearColor(0.5f,0.5f,0.5f,1.0f);

        if (modelight==false){
            glDisable(GL_LIGHT0);
            glDisable(GL_LIGHTING);
        }else{
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
        }

        //Draw the faces
        for (int i=0;i<C.count();i++){

            glBegin(GL_POLYGON);

            glVertex3f(Paux[C[i].C1()].X(), Paux[C[i].C1()].Y(), Paux[C[i].C1()].Z());
            glVertex3f(Paux[C[i].C2()].X(), Paux[C[i].C2()].Y(), Paux[C[i].C2()].Z());
            glVertex3f(Paux[C[i].C3()].X(), Paux[C[i].C3()].Y(), Paux[C[i].C3()].Z());
            if(C[i].C4()!=-1)glVertex3f(Paux[C[i].C4()].X(), Paux[C[i].C4()].Y(), Paux[C[i].C4()].Z());

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

        DrawCube(P,C);
        DrawImagePoints();

        return;
    }
};

#endif
