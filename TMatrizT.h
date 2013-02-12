#ifndef __TMATRIZT_H__
#define __TMATRIZT_H__

#include <transformacion/FuncionesVarias.h>
using namespace FuncionesVarias;

////////////////////////////////////////////////
//               Class Color
////////////////////////////////////////////////
class TColor{
private:
    float C[4];
public:
    TColor(){
        C[0]=C[1]=C[2]=C[3]=-1;
        return;
    }
    float& R(){ return C[0]; }
    float& G(){ return C[1]; }
    float& B(){ return C[2]; }
    float& Alpha(){ return C[3]; }
};

////////////////////////////////////////////////
//               Class Cara
////////////////////////////////////////////////
class TCara{
private:
    int F[4];
public:
    TCara(){
        F[0]=F[1]=F[2]=F[3]=-1;
        return;
    }
    int& F1(){ return F[0]; }
    int& F2(){ return F[1]; }
    int& F3(){ return F[2]; }
    int& F4(){ return F[3]; }
};

////////////////////////////////////////////////
//               Class Punto3D
////////////////////////////////////////////////
class TPunto3D{
    friend class TMatrizT;
private:
    float P[4];
public:
    TPunto3D(){
        P[0]=P[1]=P[2]=0;
        P[3]=1;
        return;
    }
    TPunto3D(float x, float y, float z){
        P[0]=x; P[1]=y; P[2]=z; P[3]=1;
        return;
    }
    void ToRandom(void){
        //xyz random on [-10,10];
        P[0] = rand()%20-10;
        P[1] = rand()%20-10;
        P[2] = rand()%20-10;
        P[3] = 1;
        return;
    }
    TPunto3D operator+(TPunto3D P2){
        TPunto3D R;
        R.P[0] = P[0]+P2.P[0];
        R.P[1] = P[1]+P2.P[1];
        R.P[2] = P[2]+P2.P[2];
        R.P[3] = 1;
        return R;
    }
    TPunto3D operator-(TPunto3D P2){
        TPunto3D R;
        R.P[0] = P[0]-P2.P[0];
        R.P[1] = P[1]-P2.P[1];
        R.P[2] = P[2]-P2.P[2];
        R.P[3] = 1;
        return R;
    }
    TPunto3D operator*(float v){
        TPunto3D R;
        R.P[0] = P[0]*v;
        R.P[1] = P[1]*v;
        R.P[2] = P[2]*v;
        R.P[3] = 1;
        return R;        
    }
    TPunto3D operator/(float v){
        TPunto3D R;
        if(!Cero(v)){
            R.P[0] = P[0]/v;
            R.P[1] = P[1]/v;
            R.P[2] = P[2]/v;
            R.P[3] = 1;
        }
        return R;
    }
    friend ostream& operator<<(ostream &medio, TPunto3D p){
        medio<<"("<<p.P[0]<<","<<p.P[1]<<","<<p.P[2]<<")"<<endl;
        return(medio);
    }

    float& X(){ return P[0]; }
    float& Y(){ return P[1]; }
    float& Z(){ return P[2]; }

    void AnyadirRuido(void){
        float tamruido = 120;

        P[0]+=tamruido*((float)(rand()%100)-50)/50;
        P[1]+=tamruido*((float)(rand()%100)-50)/50;
        P[2]+=tamruido*((float)(rand()%100)-50)/50;
    }

    float Modulo(void){
        return(sqrt(P[0]*P[0]+P[1]*P[1]+P[2]*P[2]));
    }
    void Normalizar(void){
        float m = Modulo();
        if(!Cero(m)){
            P[0]/=m;
            P[1]/=m;
            P[2]/=m;
        }
        return;
    }
    //Vector product
    TPunto3D operator*(TPunto3D P2){
        TPunto3D R;
        R.P[0] = P[1]*P2.P[2]-P[2]*P2.P[1];
        R.P[1] = P[2]*P2.P[0]-P[0]*P2.P[2];
        R.P[2] = P[0]*P2.P[1]-P[1]*P2.P[0];
        R.P[3] = 1;
        return R;
    }
};

////////////////////////////////////////////////
//      Class Transformation Matrix 3D
////////////////////////////////////////////////
class TMatrizT{
private:
    float M[4][4];
public:
    //Constructors
    TMatrizT(void){
        ToIdent();
        return;
    }

    TMatrizT(float rx, float ry, float rz, float tx, float ty, float tz){
        ToTransform(rx,ry,rz,tx,ty,tz);
        return;
    }

    //Coordinate system from 3 pints
    //A1 is the origin. NOT vectors, they have to be inferred.
    TMatrizT(TPunto3D A,TPunto3D B, TPunto3D C){
        TPunto3D vx = B-A;
        vx.Normalizar();

        TPunto3D vaux = C-A;
        vaux.Normalizar();

        TPunto3D vy = vx*vaux;
        vy.Normalizar();
        TPunto3D vz = vy*vx;
        vz.Normalizar();

        M[0][0]=vx.X(); M[0][1]=vy.X(); M[0][2]=vz.X();     M[0][3]=A.X();
        M[1][0]=vx.Y(); M[1][1]=vy.Y(); M[1][2]=vz.Y();     M[1][3]=A.Y();
        M[2][0]=vx.Z(); M[2][1]=vy.Z(); M[2][2]=vz.Z();     M[2][3]=A.Z();
        M[3][0]=0;      M[3][1]=0;      M[3][2]=0;          M[3][3]=1;

        return;
    }

    //Computes the difference between the three first columns
    float Diff1(TMatrizT M2){
        float d = 0;
        for(int i=0;i<4;i++){
            for(int j=0;j<3;j++){
                d+=fabs(M[i][j]-M2.M[i][j]);
            }
        }
        return(d);
    }
    //Computes the difference of the last column
    float Diff2(TMatrizT M2){
        float d = 0;
        for(int i=0;i<4;i++){
            d+=fabs(M[i][3]-M2.M[i][3]);
        }
        return(d);
    }

    //Operators
    TMatrizT operator*(TMatrizT M2){
        TMatrizT R;
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                float r=0;
                for(int k=0;k<4;k++){
                    r += M[i][k]*M2.M[k][j];
                }
                R.M[i][j] = r;
            }
        }
        return R;
    }

    void ToIdent(void){
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                if(i==j) M[i][j]=1;
                else M[i][j]=0;
            }
        }
        return;
    }

    TPunto3D operator*(TPunto3D p){
        TPunto3D R;
        for(int i=0;i<4;i++){
            R.P[i]=0;
            for(int j=0;j<4;j++){
                R.P[i]+=M[i][j]*p.P[j];
            }
        }
        return R;
    }

    friend ostream& operator<<(ostream &medio, TMatrizT m){
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                medio<<m.M[i][j]<<" ";
            }
            medio<<endl;
        }
        return(medio);
    }

    void ToRotX(float r){
        ToIdent();
        float s=(float)sin(r);
        float c=(float)cos(r);
        M[1][1]=c;
        M[1][2]=-s;
        M[2][1]=s;
        M[2][2]=c;
        return;
    }

    void ToRotY(float r){
        ToIdent();
        float s=(float)sin(r);
        float c=(float)cos(r);
        M[0][0]=c;
        M[0][2]=s;
        M[2][0]=-s;
        M[2][2]=c;
        return;
    }

    void ToRotZ(float r){
        ToIdent();
        float s=(float)sin(r);
        float c=(float)cos(r);
        M[0][0]=c;
        M[0][1]=-s;
        M[1][0]=s;
        M[1][1]=c;
        return;
    }

    void ToRot(float rx, float ry, float rz){
        TMatrizT Rx, Ry, Rz, Tras;
        Rx.ToRotX(rx);
        Ry.ToRotY(ry);
        Rz.ToRotZ(rz);

        (*this) = Rz*Ry*Rx;

        return;
    }
    void ToTras(float tx, float ty, float tz){
        ToIdent();
        M[0][3]=tx;
        M[1][3]=ty;
        M[2][3]=tz;
        return;
    }
    void ToTransform(float rx, float ry, float rz, float tx, float ty, float tz){
        TMatrizT R;
        R.ToRot(rx,ry,rz);
        TMatrizT T;
        T.ToTras(tx,ty,tz);
        (*this)=T*R;
        return;
    }

    void Transformar(float &x, float &y, float &z){
        TPunto3D P(x,y,z);
        P = (*this)*P;
        x=P.X();
        y=P.Y();
        z=P.Z();
        return;
    }
    void Transformar(TPunto3D &P){
        P=(*this)*P;
        return;
    }

    void InvertTras(void){
        M[0][3]=-M[0][3];
        M[1][3]=-M[1][3];
        M[2][3]=-M[2][3];
        return;
    }

    void Invert(void){
        int i,j;
        for(i=0;i<3;i++){
            for(j=i+1;j<3;j++){
                float aux=M[i][j];
                M[i][j]=M[j][i];
                M[j][i]=aux;
            }
        }
        float tx,ty,tz;
        tx=M[0][3];
        ty=M[1][3];
        tz=M[2][3];
        M[0][3]=-(M[0][0]*tx+M[0][1]*ty+M[0][2]*tz);
        M[1][3]=-(M[1][0]*tx+M[1][1]*ty+M[1][2]*tz);
        M[2][3]=-(M[2][0]*tx+M[2][1]*ty+M[2][2]*tz);
        return;
    }

    bool From3Matches(TPunto3D A1,TPunto3D A2, TPunto3D A3, TPunto3D B1, TPunto3D B2, TPunto3D B3){
        TMatrizT C1 = TMatrizT(A1,A2,A3); //First coordinate system
        TMatrizT C2 = TMatrizT(B1,B2,B3); //Second coordinate system
        C2.Invert(); //Swap between the second coordinate system and the reference system
        (*this) = C1*C2; //Swap from the second coordinate system to the first
        return true;
    }

    void GetTranslation(float &tx, float &ty, float &tz){
        tx = M[0][3];
        ty = M[1][3];
        tz = M[2][3];
        return;
    }

};


#endif
