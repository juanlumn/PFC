#ifndef TCAMARAKINECT_H
#define TCAMARAKINECT_H

#include <QImage>
#include <QPainter>
#include <QList>
#include <QList>
#include <QTime>

#include <XnOS.h>
#include <XnCppWrapper.h>

#include <vector>
#include "surflib.h"

#include <FASTLib/fast.h>

//Library for RANSAC
#include "transformacion/TMatrizT.h"

using namespace surf;
using namespace xn;

const int NFS=64;
const float INF=100000000;

class TEmparejado{
public:
    //A appends the number of F1 and B the number of F2
    int A,B;
    TEmparejado(int a, int b){
        A=a;
        B=b;
        return;
    }
};

class TPixel3D{
public:
    unsigned char r,g,b;
    float x,y,z;
    int d;
};

class TFeature{
public:
    float des[NFS];
    float x,y;
    float ang;
    float lap;
    float sco;
    TPixel3D pix;

    //Methods
    TFeature(){
        x=y=ang=lap=sco=0;
    }
    TFeature(Ipoint &p,TPixel3D pixel){
        pix=pixel;
        x = p.x;
        y = p.y;
        sco = p.scale;
        lap = p.laplace;
        ang = p.ori;

        for(int i=0;i<NFS;i++) des[i] = p.ivec[i];
        return;
    }
    float DistanceEuc(TFeature &f2){
        if(lap * f2.lap < 0) return(INF);
        float d=0;
        for(int i=0;i<NFS;i++){
            float v1=des[i];
            float v2=f2.des[i];
            d+=(v1-v2)*(v1-v2);
        }
        return(d);
    }
};

class TImagen3D{
private:
    TPixel3D *Data;
    QList<TFeature> Features;
    int width,height,tam;
public:

    //Void constructor
    TImagen3D(void){
        Data=NULL;
        Features.clear();
        width = height = tam = 0;
        return;
    }
    //Input parameters constructor's
    TImagen3D(int w, int h){
        width = w;
        height = h;
        tam = width * height;
        Data = new TPixel3D[tam];
        return;
    }

    //Copy constructor
    TImagen3D(const TImagen3D &orig){
        Data=NULL;
        width = height = tam = 0;
        if(orig.Data!=NULL){
            width = orig.width;
            height = orig.height;
            tam = width*height;
            Data = new TPixel3D[tam];
            TPixel3D *porig = orig.Data;
            TPixel3D *pdest = Data;
            for(int i=0;i<tam;i++){
                *pdest = *porig;
                pdest++;
                porig++;
            }
        }
        Features = orig.Features;
        return;
    }
    //Equal operator
    void operator=(TImagen3D orig){
        if(width!=orig.width || height!=orig.height ){
            if(Data) delete []Data;
            width = orig.width;
            height = orig.height;
            tam = width*height;
            if(orig.Data) Data = new TPixel3D[tam];
            else Data = NULL;
        }
        if(orig.Data!=NULL){
            TPixel3D *porig = orig.Data;
            TPixel3D *pdest = Data;
            for(int i=0;i<tam;i++){
                *pdest = *porig;
                pdest++;
                porig++;
            }
        }
        Features = orig.Features;
        return;
    }
    //Destructor
    ~TImagen3D(){
        if(Data){
            delete []Data;
        }
        return;
    }

    int Tam(void){ return tam; }
    int Width(void){ return width; }
    int Height(void){ return height; }

    //Sets a determined pixel
    void SetPixel(TPixel3D p, int i, int j){
        Data[i*width+j]=p;
        return;
    }
    //Gets a determined pixel
    TPixel3D &GetPixel(int i, int j){
        return(Data[i*width+j]);
    }
    //Returns the full vector
    TPixel3D *GetData(){
        return(Data);
    }
    //Returns the full size
    int GetTam(void){
        return width*height;
    }

    //Turns into RGB 888 every single Image frame
    QImage GetQImage(void){
        QImage resul(width,height,QImage::Format_RGB888);
        TPixel3D *psource = Data;
        for(int y=0;y<height;y++){
            uchar *pdest = resul.scanLine(y);
            for(int x=0;x<width;x++){
                *pdest = (psource->r); pdest++;
                *pdest = (psource->g); pdest++;
                *pdest = (psource->b); pdest++;
                psource++;
            }
        }
        return resul;
    }
    //Turns into RGB 888 every single Deep frame
    QImage DeepQImage(void){
        QImage resul(width,height,QImage::Format_RGB888);
        TPixel3D *psource = Data;
        int maxd=-1;
        for(int y=0;y<height;y++){
            for(int x=0;x<width;x++){
                if (psource->d>maxd)maxd=psource->d;
                psource++;
            }
        }
        psource = Data;
        for(int y=0;y<height;y++){
            uchar *pdest = resul.scanLine(y);
            for(int x=0;x<width;x++){
                if (psource->d>-1){
                    int valor=(psource->d*256/maxd);
                    *pdest = valor; pdest++;
                    *pdest = valor; pdest++;
                    *pdest = 0; pdest++;
                }
                else{
                    *pdest = 0; pdest++;
                    *pdest = 0; pdest++;
                    *pdest = 0; pdest++;
                }
                psource++;
            }
        }
        return resul;
    }
    //Returns the blend of two images
    QImage GetQImageFundida(TImagen3D &img){
        QImage resul(width,height,QImage::Format_RGB888);
        TPixel3D *psource1 = Data;
        TPixel3D *psource2 = img.Data;
        for(int y=0;y<height;y++){
            uchar *pdest = resul.scanLine(y);
            for(int x=0;x<width;x++){
                if(psource1->d <= psource2->d){
                    *pdest = (psource1->r); pdest++;
                    *pdest = (psource1->g); pdest++;
                    *pdest = psource1->b; pdest++;
                }else{
                    *pdest = (psource2->r); pdest++;
                    *pdest = (psource2->g); pdest++;
                    *pdest = psource2->b; pdest++;
                }
                psource1++;
                psource2++;
            }
        }
        return resul;
    }

    void ComputeFeatures(void){
        //Configuration
        bool doubleImageSize = false;
        bool upright = true;
        bool extended = false; //128 features
        int indexSize = 4;

        //Original Picture
        byte *imfast = new byte[width*height];
        byte *imfastp = imfast;
        Image *im = new Image(width,height);
        double **dim = im->getPixels();
        for(int y=0;y<height;y++){
            TPixel3D *line = &Data[y*width];
            double *drow = dim[y];
            for(int x=0;x<width;x++){
                int v = (line->r+line->g+line->b)/3;
                *drow = v;
                *imfastp = v; imfastp++;
                drow++;
                line++;
            }
        }

        //FAST (selector)
        int numfastpoints=0; //X=9, 10, 11 o 12
        int w = width;
        int h = height;

        xy *fastpoints = fast12_detect_nonmax((const byte*)imfast,w,h,w,25,&numfastpoints);

        //SURF (descriptor)
        vector< Ipoint > ipts;
        ipts.reserve(numfastpoints);
        for(int i=0;i<numfastpoints;i++){
            Ipoint p;
            p.x = fastpoints[i].x;
            p.y = fastpoints[i].y;
            p.scale = 1;
            ipts.push_back(p);
        }

        Image iimage(im, doubleImageSize);
        Surf des(&iimage, doubleImageSize, upright, extended, indexSize);

        //Orientations
        for (unsigned n=0; n<ipts.size(); n++){
            des.setIpoint(&ipts[n]);
            des.assignOrientation();
            des.makeDescriptor();
        }

        Features.clear();
        for (unsigned n=0; n<ipts.size(); n++){
            TPixel3D pixel = GetPixel((int)ipts[n].y,(int)ipts[n].x);
            if(pixel.d>0){
                Features.append(TFeature(ipts[n],pixel));
            }
        }

        delete im;
        delete imfast;
        free(fastpoints);

        return;
    }
    //Returns the features
    QList <TFeature> returnFeatures(){
        return Features;
    }

    QImage GetImageWithFeatures(void){

        QImage I = this->GetQImage();
        QPainter p(&I);
        p.setPen(QPen(Qt::red));
        p.setBrush(QBrush(Qt::red));
        for(int i=0;i<Features.count();i++){
            p.drawEllipse(Features[i].x-1,Features[i].y-1,3,3);
        }
        p.end();

        return I;
    }
    //Compute the Pairings
    QList <TEmparejado> Emparejamientos(QList <TFeature> &F1,QList <TFeature> &F2){
        //Two int vectors wich are going to save the pairings, initialized on -1
        QVector <int> P1(F1.count()),P2(F2.count());
        for (int i=0;i<P1.count();i++)P1[i]=-1;
        for (int i=0;i<P2.count();i++)P2[i]=-1;
        //Distance vector (F1xF2)
        float distancias[F1.count()][F2.count()];

        //Computes the Euclidean distance from F1 to F2 and saves it in the distances vector
        for (int i=0;i<F1.count();i++){
            for (int j=0;j<F2.count();j++)distancias[i][j] = F1[i].DistanceEuc(F2[j]);
        }

        //Reads the distances from F1 to F2 and saves the smaller in P1
        for (int i=0;i<F1.count();i++){
            float d1=INF,d2=INF;
            for (int j=0;j<F2.count();j++){
                if (distancias[i][j]<=d1){
                    if (d1<d2)d2=d1;
                    d1=distancias[i][j];
                    P1[i]=j;
                }
            }
            float rate=d1/d2;
            if (rate>0.85)P1[i]=-1;
        }
        //Reads the distances from F2 to F1 and saves the smaller in P2
        for (int i=0;i<F2.count();i++){
            float d1=INF,d2=INF;
            for (int j=0;j<F1.count();j++){
                if (distancias[j][i]<=d1){
                    if (d1<d2)d2=d1;
                    d1=distancias[j][i];
                    P2[i]=j;
                }
            }
            float rate=d1/d2;
            if (rate>0.85)P2[i]=-1;
        }
        //Creates a vector with the pairing points
        int potenciales=0;
        QList<TEmparejado> Emparejados;
        for(int i=0;i<F1.count();i++){
            if (P1[i]!=-1){
                if (P2[P1[i]]==i)Emparejados.append(TEmparejado(i,P1[i]));
                potenciales++;
            }

        }
        return Emparejados;
    }

    //Computes three different groups of TPunto3D points called "Terna" from the reference Image
    QList <TPunto3D> Terna(QList <TFeature> &F1,QList <TFeature> &F2,QList <TEmparejado> &E){
        QList<TPunto3D> Ternas;
        if (E.count()>3){
            int a=0,b=0,c=0;
            do{
                a=rand()%E.count();
                b=rand()%E.count();
                c=rand()%E.count();
            }while(a==b || a==c || b==c);
            Ternas.append(TPunto3D(F2[E[a].B].pix.x,F2[E[a].B].pix.y,F2[E[a].B].pix.z));
            Ternas.append(TPunto3D(F2[E[b].B].pix.x,F2[E[b].B].pix.y,F2[E[b].B].pix.z));
            Ternas.append(TPunto3D(F2[E[c].B].pix.x,F2[E[c].B].pix.y,F2[E[c].B].pix.z));

            Ternas.append(TPunto3D(F1[E[a].A].pix.x,F1[E[a].A].pix.y,F1[E[a].A].pix.z));
            Ternas.append(TPunto3D(F1[E[b].A].pix.x,F1[E[b].A].pix.y,F1[E[b].A].pix.z));
            Ternas.append(TPunto3D(F1[E[c].A].pix.x,F1[E[c].A].pix.y,F1[E[c].A].pix.z));
        }
        return Ternas;
    }
    //Computes the final "Terna"
    QList <TPunto3D> TernaFinal(TMatrizT &M,QList <TFeature> &F1, QList <TFeature> &F2,QList <TEmparejado> &E){

        QList<TPunto3D> Ternas;
        QList <TFeature> F=F1;

        //Applies the transformation to the refresh points
        for (int i=0;i<F.count();i++){
            TPunto3D punto=TPunto3D(F[i].pix.x,F[i].pix.y,F[i].pix.z);
            M.Transformar(punto);
            F[i].pix.x=punto.X();
            F[i].pix.y=punto.Y();
            F[i].pix.z=punto.Z();
        }

        TPunto3D A2,B2,C2,A1,B1,C1;
        float na=0,nb=0,nc=0;

        //Computes the cuadratic Euclidean distance
        for (int i=0;i<E.count();i++){
            float Ax=F2[E[i].B].pix.x;
            float Ay=F2[E[i].B].pix.y;
            float Az=F2[E[i].B].pix.z;

            float Bx=F[E[i].A].pix.x;
            float By=F[E[i].A].pix.y;
            float Bz=F[E[i].A].pix.z;

            float d=(((Ax-Bx)*(Ax-Bx))+((Ay-By)*(Ay-By))+((Az-Bz)*(Az-Bz)));

            //If the cuadratic Euclidean distance between A and B1 is smaller than "d"
            if (d<=0.0001){
                float w;

                w = F2[E[i].B].pix.x/(1+d);
                A2 = A2 + TPunto3D(F2[E[i].B].pix.x,F2[E[i].B].pix.y,F2[E[i].B].pix.z)*w;
                A1 = A1 + TPunto3D(F1[E[i].A].pix.x,F1[E[i].A].pix.y,F1[E[i].A].pix.z)*w;
                na = na + w;

                w = F2[E[i].B].pix.y/(1+d);
                B2 = B2 + TPunto3D(F2[E[i].B].pix.x,F2[E[i].B].pix.y,F2[E[i].B].pix.z)*w;
                B1 = B1 + TPunto3D(F1[E[i].A].pix.x,F1[E[i].A].pix.y,F1[E[i].A].pix.z)*w;
                nb = nb + w;

                w = F2[E[i].B].pix.z/(1+d);
                C2 = C2 + TPunto3D(F2[E[i].B].pix.x,F2[E[i].B].pix.y,F2[E[i].B].pix.z)*w;
                C1 = C1 + TPunto3D(F1[E[i].A].pix.x,F1[E[i].A].pix.y,F1[E[i].A].pix.z)*w;
                nc = nc + w;
            }
        }
        if(na>0){
            A2=A2/na;
            A1=A1/na;
        }
        if(nb>0){
            B2=B2/nb;
            B1=B1/nb;
        }
        if(nc>0){
            C2=C2/nc;
            C1=C1/nc;
        }
        Ternas.append(A2);
        Ternas.append(B2);
        Ternas.append(C2);

        Ternas.append(A1);
        Ternas.append(B1);
        Ternas.append(C1);

        return Ternas;
    }
    //Computes the transformation
    float Transformacion(TMatrizT &M,QList <TFeature> &F1, QList <TFeature> &F2,QList <TEmparejado> &E){
        float cuenta=0;
        QList <TFeature> F=F1;

        //Applies the transformation to the refresh points
        for (int i=0;i<F.count();i++){
            TPunto3D punto=TPunto3D(F[i].pix.x,F[i].pix.y,F[i].pix.z);
            M.Transformar(punto);
            F[i].pix.x=punto.X();
            F[i].pix.y=punto.Y();
            F[i].pix.z=punto.Z();
        }

        //Computes the cuadratic Euclidean distance
        for (int i=0;i<E.count();i++){
            float Ax=F2[E[i].B].pix.x;
            float Ay=F2[E[i].B].pix.y;
            float Az=F2[E[i].B].pix.z;

            float Bx=F[E[i].A].pix.x;
            float By=F[E[i].A].pix.y;
            float Bz=F[E[i].A].pix.z;

            float d=INF;

            d=(((Ax-Bx)*(Ax-Bx))+((Ay-By)*(Ay-By))+((Az-Bz)*(Az-Bz)));

            //If the cuadratic Euclidean distance between A and B1 is smaller than "d"
            if (d<=0.0001) cuenta++;
        }
        return cuenta;
    }

    //Applies the best transformation to all the refresh points
    void AplicaModelo(TMatrizT &M1){
        TPixel3D *Datos=Data;
        //For all Data
        for(int i=0;i<tam;i++){
            //If there is deep information
            if (Datos->d>-1){
                //Transforms the point
                TPunto3D punto=TPunto3D(Datos->x,Datos->y,Datos->z);
                M1.Transformar(punto);
                Datos->x=punto.X();
                Datos->y=punto.Y();
                Datos->z=punto.Z();
            }
            Datos++;
        }
        return;
    }
    void BestE(QList <TImagen3D> &LK,int &BestKey,QList<TEmparejado> &E){

        E.clear();
        BestKey=0;

        QList <TFeature> F1, F2;
        int contador=0;
        //Computes the SURF points from the refresh image and saves it in F1
        F1.clear();
        F1=returnFeatures();
        for (int i=0;i<LK.count();i++){
            //Computes the SURF points from the reference image and saves it in F2
            F2.clear();
            F2=LK[i].returnFeatures();
            if (F2.count()!=0){
                QList<TEmparejado> E1=Emparejamientos(F1,F2);
                if(contador<E1.count()){
                    contador=E1.count();
                    //Stores the best keyframe and pairing
                    BestKey=i;
                    E.clear();
                    E=E1;
                }
            }
        }

        return;
    }


    //RANSAC Method
    QImage ransac(TImagen3D &LK,TMatrizT &M1,int &N,QList<TEmparejado> E){
        N=0;
        QList <TFeature> F1,F2;
        F2.clear();
        //Computes the SURF points from the reference image and saves it in F2
        F2=LK.returnFeatures();
        //Computes the SURF points from the refresh image and saves it in F1
        F1=returnFeatures();
        //Feature's Image
        QImage img = GetImageWithFeatures();
        if (F2.count()!=0 && E.count()>3){
            int niter=0;
            //Douring 100 iterations
            while (niter<100){
                QList<TPunto3D> A=Terna(F1,F2,E);
                TMatrizT M;
                if (M.From3Matches(A[0],A[1],A[2],A[3],A[4],A[5])==1){
                    float cuenta=Transformacion(M,F1,F2,E);
                    if (cuenta>N){
                        N=cuenta;
                        niter=0;
                        M1=M;
                    }else niter++;
                }else niter++;
                A.clear();
            }

            //Final pairings in green
            QPainter p(&img);
            p.setBrush(Qt::green);
            p.setPen(Qt::green);
            for(int i=0;i<E.count();i++)p.drawRect(F1[E[i].A].x,F1[E[i].A].y,3,3);
            p.end();

        }
        return img;
    }
    //Draws a red shape in the picture
    void ultimaimg(){
        for (int j=0;j<this->width;j++){
            for (int i=0;i<this->height;i++){
                TPixel3D pixel=this->GetPixel(i,j);
                pixel.r=INF;
                pixel.b=NULL;
                pixel.g=NULL;
                if(i<=30||j<=20||(i>(this->height)-15&&i<this->height)||(j>(this->width)-35&&j<this->width))this->SetPixel(pixel,i,j);
            }
        }

        return;
    }

    float CataCentro(){
        int tv = 7;
        int cx = width/2;
        int cy = height/2;

        float dmedia=0;
        int nelem=0;

        for(int x=cx-tv;x<=cx+tv;x++){
            for(int y=cy-tv;y<=cy+tv;y++){
                TPixel3D pix = GetPixel(y,x);
                if(pix.d>0){
                    dmedia += sqrt(pix.x*pix.x+pix.y*pix.y+pix.z*pix.z);
                    nelem++;
                }
            }
        }
        if(nelem>0) dmedia/=nelem;
        return dmedia;
    }
};

class TCamaraKinect{
private:
    Context g_context;
    DepthGenerator g_depth;
    ImageGenerator g_image;
    DepthMetaData g_depthMD;
    ImageMetaData g_imageMD;
    //Own buffers
    int TamBuffer;
    TImagen3D *Buffer;
    XnPoint3D *p_source,*p_dest;
public:
    TCamaraKinect(){
        XnStatus rc;
        rc = g_context.Init();
        if (rc != XN_STATUS_OK)return;
        g_depth.Create(g_context);
        g_image.Create(g_context);
        //Resolution and fps
        XnMapOutputMode mapMode;
        mapMode.nXRes = XN_VGA_X_RES;
        mapMode.nYRes = XN_VGA_Y_RES;
        mapMode.nFPS = 30;
        g_image.SetMapOutputMode(mapMode);
        g_depth.SetMapOutputMode(mapMode);
        g_context.StartGeneratingAll();
        //Mirror
        g_context.SetGlobalMirror(false);

        Buffer = NULL;
        if(!ReadNewFrame()) return;

        if (g_imageMD.FullXRes() != g_depthMD.FullXRes() || g_imageMD.FullYRes() != g_depthMD.FullYRes()) return;
        if (g_imageMD.PixelFormat() != XN_PIXEL_FORMAT_RGB24) return;

        return;
    }
    //Destructor
    ~TCamaraKinect(){
        if(Buffer) delete Buffer;
        if(p_source) delete []p_source;
        if(p_dest) delete []p_dest;
        g_context.StopGeneratingAll();
        //g_context.Shutdown();
        g_depthMD.Free();
        g_imageMD.Free();
    }
    //Image to world coordinates
    bool ImageToWorldCoord(float &x, float &y, float &z){
        p_source->X=x;
        p_source->Y=y;
        p_source->Z=z;
        XnStatus s = g_depth.ConvertProjectiveToRealWorld(1,p_source,p_dest);
        x=p_dest->X;
        y=p_dest->Y;
        z=p_dest->Z;
        return(s==XN_STATUS_OK);
    }
    //World to image coordinates
    bool WorldToImageCoord(float &x, float &y, float &z){
        p_source->X=x;
        p_source->Y=y;
        p_source->Z=z;
        XnStatus s = g_depth.ConvertRealWorldToProjective(1,p_source,p_dest);
        x=p_dest->X;
        y=p_dest->Y;
        z=p_dest->Z;
        return(s==XN_STATUS_OK);
    }
    //New frame
    bool ReadNewFrame(void){
        XnStatus rc;
        rc = g_context.WaitAnyUpdateAll();
        if (rc != XN_STATUS_OK) return(false);

        g_depth.GetMetaData(g_depthMD);
        g_image.GetMetaData(g_imageMD);

        //Depth reprojection over the rgb image
        g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);

        //Buffer information
        const XnRGB24Pixel* pimage;
        pimage = g_imageMD.RGB24Data();
        const XnDepthPixel* pdepth = g_depthMD.Data();

        if(!Buffer){
            //Buffer's size
            TamBuffer = g_depthMD.FullXRes()*g_depthMD.FullYRes();
            Buffer = new TImagen3D(g_depthMD.FullXRes(),g_depthMD.FullYRes());
            p_source = new XnPoint3D[TamBuffer];
            p_dest = new XnPoint3D[TamBuffer];
        }

        TPixel3D *bact = Buffer->GetData();
        int np=0;
        XnPoint3D *ps=p_source;
        int w = g_depthMD.FullXRes();

        for(int x=0;x<TamBuffer;x++){
            bact->r = pimage->nRed;
            bact->g = pimage->nGreen;
            bact->b = pimage->nBlue;
            bact->d = *pdepth;
            if(bact->d<=0) bact->d=-1;
            if(bact->d>0){
                ps->X=(XnFloat)(x%w);
                ps->Y=(XnFloat)(x/w);
                ps->Z=(XnFloat)bact->d;
                ps++;
                np++;
            }
            bact++;
            pimage++;
            pdepth++;
        }
        //Depth to xyz
        g_depth.ConvertProjectiveToRealWorld(np,p_source,p_dest);
        bact = Buffer->GetData();
        XnPoint3D *pd = p_dest;
        for(int x=0;x<TamBuffer;x++){
            if(bact->d>0){
                bact->x = pd->X/1000;
                bact->y = pd->Y/1000;
                bact->z = -pd->Z/1000;
                pd++;
            }
            bact++;
        }
        Buffer->ComputeFeatures();
        return(true);
    }
    //Data Buffer pointer
    TImagen3D* GetImageBuffer(void){
        return(Buffer);
    }
    //Width of the buffer
    int GetTamBuffer(void){
        return(TamBuffer);
    }
    //Working resolution
    bool GetResolution(int &width, int &height){
        width = g_imageMD.FullXRes();
        height = g_imageMD.FullYRes();
        return(true);
    }

    void DibujarImagen3D(QImage &referencia, TImagen3D &img, TMatrizT M, float zoom){

        //From img.Data to p_source
        TPixel3D *datos=img.GetData();
        XnPoint3D *ps = p_source;

        int nelem = 0;
        for (int i=0;i<img.Tam();i++){
            if(datos->d>0){
                TPunto3D punto=TPunto3D(datos->x, datos->y, datos->z);
                M.Transformar(punto);

                ps->X=(XnFloat)(punto.X()*1000);
                ps->Y=(XnFloat)(punto.Y()*1000);//cuidado
                ps->Z=(XnFloat)(-punto.Z()*1000);

                ps++;
                nelem++;
            }
            datos++;
        }
        //Reprojection
        g_depth.ConvertRealWorldToProjective(nelem,p_source,p_dest);

        //Gets the reprojections from p_dest
        XnPoint3D *pd = p_dest;
        datos=img.GetData();
        for (int i=0;i<img.Tam();i++){
            if(datos->d>=0){
                float x,y,d,r,g,b;
                x = pd->X;
                y = pd->Y;

                //Center and escalate
                x = (x-img.Width()/2)*zoom + referencia.width()/2;
                y = (y-img.Height()/2)*zoom + referencia.height()/2;

                d = pd->Z;
                r = datos->r;
                g = datos->g;
                b = datos->b;

                if (x<referencia.width() && y<referencia.height()&&x>=0&&y>=0)referencia.setPixel(x,y,qRgb(r,g,b));
                pd++;
            }
            datos++;
        }
        return;
    }
};

#endif // TCAMARAKINECT_H
