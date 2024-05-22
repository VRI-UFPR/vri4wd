/*
KNN WLAN LOCALIZATION
by Carlos Eduardo Magrin

VRI4WD UFPR-MAP 
https://github.com/VRI-UFPR/ufpr-map
*/

#include "ros/ros.h"
#include "std_msgs/Float32.h" 
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "kbhit.h" 

#define TAM_TRAIN 134 //training(rows)
#define TAM_CLASS 1 //sensor readings(rows)
#define TAM_ATRIB 19 //features WITHOUT classifier (columns)
#define TAM_ATRIB_AMB 20 //environment, path
#define TAM_ATRIB_GRID 21 //landmarks
#define QTD_AMB 6 //number of environments (paths)
#define QTD_GRID 123 //number of landmarks

#define TAM_K 3 //k size

#define QTD_WIFI 19 //access points (wlan)

ros::Publisher pub_gridMaior;
ros::Publisher pub_gridMenor;

int grid_maior, grid_menor;

using namespace std;

char tecla = '0';

int rssi1_wlan, rssi2_wlan, rssi3_wlan, rssi4_wlan, rssi5_wlan, rssi6_wlan, rssi7_wlan, rssi8_wlan, rssi9_wlan, rssi10_wlan, rssi11_wlan, rssi12_wlan, rssi13_wlan, rssi14_wlan, rssi15_wlan, rssi16_wlan, rssi17_wlan, rssi18_wlan, rssi19_wlan;

float robotOrientation; 

void subCallback1(const std_msgs::Int8::ConstPtr& s1){
	rssi1_wlan = s1->data; 	
}
void subCallback2(const std_msgs::Int8::ConstPtr& s2){
	rssi2_wlan = s2->data; 	
}
void subCallback3(const std_msgs::Int8::ConstPtr& s3){
	rssi3_wlan = s3->data; 	
}
void subCallback4(const std_msgs::Int8::ConstPtr& s4){
	rssi4_wlan = s4->data; 	
}
void subCallback5(const std_msgs::Int8::ConstPtr& s5){
	rssi5_wlan = s5->data; 	
}
void subCallback6(const std_msgs::Int8::ConstPtr& s6){
	rssi6_wlan = s6->data; 	
}
void subCallback7(const std_msgs::Int8::ConstPtr& s7){
	rssi7_wlan = s7->data; 	
}
void subCallback8(const std_msgs::Int8::ConstPtr& s8){
	rssi8_wlan = s8->data; 	
}
void subCallback9(const std_msgs::Int8::ConstPtr& s9){
	rssi9_wlan = s9->data; 	
}
void subCallback10(const std_msgs::Int8::ConstPtr& s10){
	rssi10_wlan = s10->data; 	
}
void subCallback11(const std_msgs::Int8::ConstPtr& s11){
	rssi11_wlan = s11->data; 	
}
void subCallback12(const std_msgs::Int8::ConstPtr& s12){
	rssi12_wlan = s12->data; 	
}
void subCallback13(const std_msgs::Int8::ConstPtr& s13){
	rssi13_wlan = s13->data; 	
}
void subCallback14(const std_msgs::Int8::ConstPtr& s14){
	rssi14_wlan = s14->data; 	
}
void subCallback15(const std_msgs::Int8::ConstPtr& s15){
	rssi15_wlan = s15->data; 	
}
void subCallback16(const std_msgs::Int8::ConstPtr& s16){
	rssi16_wlan = s16->data; 	
}
void subCallback17(const std_msgs::Int8::ConstPtr& s17){
	rssi17_wlan = s17->data; 	
}
void subCallback18(const std_msgs::Int8::ConstPtr& s18){
	rssi18_wlan = s18->data; 	
}
void subCallback19(const std_msgs::Int8::ConstPtr& s19){
	rssi19_wlan = s19->data; 	
}

void ordena(double vizinhos[][2]){
	long int i,j;
	double aux=0, aux2=0;

	for(i=0;i<TAM_TRAIN;i++){
		for(j=i+1;j<TAM_TRAIN;j++){
			if(vizinhos[i][0] > vizinhos[j][0]){
				aux=vizinhos[i][0];
		    aux2=vizinhos[i][1];
		    vizinhos[i][0]=vizinhos[j][0];
		    vizinhos[i][1]=vizinhos[j][1];
		    vizinhos[j][0]=aux;
		    vizinhos[j][1]=aux2;
		  }
    }
  }
}

void quickSort(double vizinhos[][2], long int esquerda, long direita)
{
  long int i, j;
  double aux,aux2,x;
  i = esquerda;
  j = direita;
  x = vizinhos[(esquerda + direita) / 2][0];
  while(i <= j)
  {
    while(vizinhos[i][0] < x && i < direita)
    {
      i++;
    }
    while(vizinhos[j][0] > x && j > esquerda)
    {
      j--;
    }
    if(i <= j)
    {

      aux=vizinhos[i][0];
      aux2=vizinhos[i][1];
      vizinhos[i][0]=vizinhos[j][0];
      vizinhos[i][1]=vizinhos[j][1];
      vizinhos[j][0]=aux;
      vizinhos[j][1]=aux2;
      i++;
      j--;
    }
  }
  if(j > esquerda)
  {
    quickSort(vizinhos, esquerda, j);
  }
  if(i < direita)
  {
    quickSort(vizinhos,  i, direita);
  }
}

int knn(double sensor[]){
  int j=0, p=0,v=0,cont=1,vot_emp=0,ambiente=0, votacao[QTD_AMB],votacao2[QTD_GRID];
  long int i=0,w=0,aux=0,aux2=0;
  int escolhido=0,escolhido2=0,temp=0,acerto=0,tempoc=0,tempot=0,tempoa=0,teste=0,r=0,tam_grid=0,idsonar=0,idwifi=0;
  char train[150] = "/home/magrin/catkin_ws/src/cognitive_blended_sensors/dataset/wlan_fingerprintMap_rawData.csv", *psRetorno,linha[50000];
  
  double treinamento[TAM_TRAIN][TAM_ATRIB_GRID], vizinhos[TAM_TRAIN][2],vizinhos2[TAM_TRAIN][2];
  double dist,dist2,result=0,result2=0,tempo=0;
  FILE *arq_train;
  
  //float peso_wifi[19]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  float peso_wifi[19]={0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0}; 
  
  if ((arq_train = fopen(train, "r"))==NULL){
		puts ("OPENING FILE ERROR");
  }

  //read training files into memory 		
  while((fgets(linha, 50000, arq_train) != NULL)){
		psRetorno = strtok(linha,   " ");
    treinamento[i][j]=atof(psRetorno);

	j++;

    do{
    	psRetorno = strtok(NULL," "); //searches for white space.
      if(j!=TAM_ATRIB_GRID){
	      treinamento[i][j]=atof(psRetorno);
        j++;
      }
    }while(psRetorno);
 
    i++;
    j=0;
  }
  
  fclose(arq_train);
  
  i=0; 
  dist=0;

  aux=TAM_CLASS;
  aux2=TAM_TRAIN;
	 
  for(i=0;i<aux;i++)
  {
    cont++;
    if(cont==1000){
      printf("LINHA: %lu\n",i);
      cont=1;
    }

    for(w=0;w<aux2;w++)
    {
      dist=0;
      for(j=0;j<QTD_WIFI;j++)
      {
        //weighted euclidean distance
        dist+=(((sensor[j]-treinamento[w][j])*(sensor[j]-treinamento[w][j]))*peso_wifi[j]);
		
      }
      
      result=sqrt(dist);
      vizinhos[w][0]=result;
      vizinhos[w][1]=(int)treinamento[w][TAM_ATRIB];
    }
    
  //kNN PATH
	for(w=0;w<TAM_TRAIN;w++){
		dist=0;
		for(j=0;j<QTD_WIFI;j++){
			dist+=((sensor[j]-treinamento[w][j])*(sensor[j]-treinamento[w][j]));

			//ROS_INFO("SENSOR: [%f]", sensor[0]); 	
		}	
		result=sqrt(dist);
		vizinhos[w][0]=result;
	
		vizinhos[w][1]=(int)treinamento[w][TAM_ATRIB];	
 	}

	ordena(vizinhos);

	//TRAIN PATH DISTANCE
    for(r=0;r<TAM_TRAIN;r++)
    {
        //ROS_INFO("DIST: %.4f TRAIN: %d\n",(float)vizinhos[r][0],(int)vizinhos[r][1]);

    }

    for(p=0;p<QTD_WIFI;p++)
    {
        votacao[p]=0;
    }
    for(v=0;v<TAM_K;v++)
    {
        votacao[(int)vizinhos[v][1]]++;
    }

    temp=-1;
    for(p=0;p<QTD_WIFI;p++)
    {
        if(votacao[p]>temp)
        {
            temp=votacao[p];
            escolhido=p;
            vot_emp=votacao[p];
        }
    }
    if(vot_emp==1)
    {
        escolhido=(int)vizinhos[0][1];
    }

    ambiente=escolhido;


    tam_grid=0;
    for(w=0;w<aux2;w++)
    {
        dist2=0;

        if((int)treinamento[w][TAM_ATRIB_GRID-2]==ambiente)
        {

            idsonar=0;
            idwifi=0;
            for(j=0;j<QTD_WIFI;j++)
            {
                //weighted euclidean distance
                dist2+=(((sensor[j]-treinamento[w][j])*(sensor[j]-treinamento[w][j]))*peso_wifi[idwifi]);
                idwifi++;

            }
            
            result2=sqrt(dist2);
            vizinhos2[w][0]=result2;
            vizinhos2[w][1]=(int)treinamento[w][TAM_ATRIB_GRID-1];
            
            //ROS_INFO("DIST: %.4f TRAIN: %d\n",(float)vizinhos2[tam_grid][0],(int)vizinhos2[tam_grid][1]);
            
            tam_grid++;
        }
        else
        {
            vizinhos2[w][0]=999;
            vizinhos2[w][1]=999;
        }

    }
    ordena(vizinhos2);
    //TRAIN LANDMARK DISTANCE
    
    for(r=0;r<TAM_TRAIN;r++)
    {
        if(vizinhos2[r][0]!=999){
          //ROS_INFO("DIST: %.4f TRAIN: %d\n",(float)vizinhos2[r][0],(int)vizinhos2[r][1]);   
        }
            

    }
    

    for(p=0;p<QTD_GRID;p++)
    {
        votacao2[p]=0;
    }
    for(v=0;v<TAM_K;v++)
    {
        votacao2[(int)vizinhos2[v][1]]++;
    }

    temp=-1;
    for(p=0;p<QTD_GRID;p++)
    {
        if(votacao2[p]>temp)
        {
            temp=votacao2[p];
            escolhido2=p;
            vot_emp=votacao2[p];
        }
    }
    if(vot_emp==1)
    {
        escolhido2=(int)vizinhos2[0][1];
    }

 	ROS_INFO("PATH: [%d] LANDMARK: [%d]", escolhido, escolhido2); 
	
  grid_maior = escolhido;
	grid_menor = escolhido2;

   }
   
 }
 


int main(int argc, char **argv){
	
	double sensor[20];

	ros::init(argc, argv, "cbs_level2_wlanlocalization");

	ros::NodeHandle n;

	ros::Subscriber sub1 = n.subscribe("/rssi1_wlan", 1000, subCallback1);
	ros::Subscriber sub2 = n.subscribe("/rssi2_wlan", 1000, subCallback2);
	ros::Subscriber sub3 = n.subscribe("/rssi3_wlan", 1000, subCallback3);
	ros::Subscriber sub4 = n.subscribe("/rssi4_wlan", 1000, subCallback4);
	ros::Subscriber sub5 = n.subscribe("/rssi5_wlan", 1000, subCallback5);
	ros::Subscriber sub6 = n.subscribe("/rssi6_wlan", 1000, subCallback6);
	ros::Subscriber sub7 = n.subscribe("/rssi7_wlan", 1000, subCallback7);
	ros::Subscriber sub8 = n.subscribe("/rssi8_wlan", 1000, subCallback8);
	ros::Subscriber sub9 = n.subscribe("/rssi9_wlan", 1000, subCallback9);
	ros::Subscriber sub10 = n.subscribe("/rssi10_wlan", 1000, subCallback10);
	ros::Subscriber sub11 = n.subscribe("/rssi11_wlan", 1000, subCallback11);
	ros::Subscriber sub12 = n.subscribe("/rssi12_wlan", 1000, subCallback12);
	ros::Subscriber sub13 = n.subscribe("/rssi13_wlan", 1000, subCallback13);
	ros::Subscriber sub14 = n.subscribe("/rssi14_wlan", 1000, subCallback14);
	ros::Subscriber sub15 = n.subscribe("/rssi15_wlan", 1000, subCallback15);
	ros::Subscriber sub16 = n.subscribe("/rssi16_wlan", 1000, subCallback16);
	ros::Subscriber sub17 = n.subscribe("/rssi17_wlan", 1000, subCallback17);
	ros::Subscriber sub18 = n.subscribe("/rssi18_wlan", 1000, subCallback18);
	ros::Subscriber sub19 = n.subscribe("/rssi19_wlan", 1000, subCallback19);

	ros::Publisher pub_gridMaior = n.advertise<std_msgs::Int16>("/wlan_path", 1000 );
	ros::Publisher pub_gridMenor = n.advertise<std_msgs::Int16>("/wlan_landmark", 1000 );

	ros::Rate loop_rate(10);

	system("rosservice call reset"); 

	while(ros::ok()&&!(tecla == 'p')){

		
		if (kbhit())
			tecla = getchar();

		sensor[0]=rssi1_wlan;
		sensor[1]=rssi2_wlan;
		sensor[2]=rssi3_wlan;
		sensor[3]=rssi4_wlan;
		sensor[4]=rssi5_wlan;
		sensor[5]=rssi6_wlan;
		sensor[6]=rssi7_wlan;
		sensor[7]=rssi8_wlan;
		sensor[8]=rssi9_wlan;
		sensor[9]=rssi10_wlan;
		sensor[10]=rssi11_wlan;
		sensor[11]=rssi12_wlan;
		sensor[12]=rssi13_wlan;
		sensor[13]=rssi14_wlan;
		sensor[14]=rssi15_wlan;
		sensor[15]=rssi16_wlan;
		sensor[16]=rssi17_wlan;
		sensor[17]=rssi18_wlan;
		sensor[18]=rssi19_wlan;
		
		knn(sensor);

		std_msgs::Int16 msg;
		std_msgs::Int16 msg1;
		msg.data = grid_maior;
		msg1.data = grid_menor;

		pub_gridMaior.publish(msg);
		pub_gridMenor.publish(msg1);
	
		ros::Rate loop_rate(10);
		ros::spinOnce();//This command will read and update all ROS topics.
		
	}
 	ROS_WARN("Positioning closed...");
	return 0;
}
