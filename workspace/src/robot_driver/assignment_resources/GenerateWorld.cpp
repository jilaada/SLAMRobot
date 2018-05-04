#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <ctime>
#include <math.h>
using namespace std;

const int xMAX = 1000-35; //a bit off the border
const int xMin = 0+35; //a bit off the border
const int yMAX = 300-35; //a bit off the border
const int yMin = -300+35; //a bit off the border

char isVaild(int x, int y, int r, int i, int *itemX, int *itemY, int *itemR) {
	char vaild = true;
	if (x+r > xMAX) {
		return false;//generate again
	}
	if (x-r < xMin) {
		return false;//generate again
	}
	if (y+r > yMAX) {
		return false;//generate again
	}
	if (y-r < yMin) {
		return false;//generate again
	}
	int j;
	for (j=0;j<(i+1);j++) {
		int x1 = itemX[j];
		int y1 = itemY[j];
		int r1 = itemR[j];

		int xdiff = abs(x1-x);
		int ydiff = abs(y1-y);

		if (ydiff < (r1+r+50)&& xdiff < (r1+r+50)) { //atleast 70 cm gap
			vaild = false;
			break;
		}
	}
	if (vaild) {
		itemX[i+1] = x;
		itemY[i+1] = y;
		itemR[i+1] = r;
		return true;
	}
	return false;
}

int main (int argc, char *argv[]) {
	char hasUnknown = 0;
	char generateBorder = 1;
	char generateBarrels = 1;
	char generateContainers = 1;
	int generateItems = 1;
	/*go through agruments*/
	for (int i = 1; i < argc; i++) {
		//cout << argv[i] << endl;
		if (strcmp(argv[i],"-noBorder")==0) {
			cout << "Physical border will not be generated." << endl;
			generateBorder = 0;
		}else if (strcmp(argv[i],"-noBarrels")==0) {
			cout << "Barrels will not be generated." << endl;
			generateBarrels = 0;
		}else if (strcmp(argv[i],"-noContainers")==0) {
			cout << "Containers will not be generated." << endl;
			generateContainers = 0;
		}else if (strcmp(argv[i],"-items")==0) {
			generateItems = atoi(argv[i+1]);
			i++;
			cout << generateItems << " item will be generated." << endl;
		}else{
			cout<<"Unknown argument: "<<argv[i]<<endl;
			cout<<"Please try again."<<endl;
			hasUnknown = 1;
			break;
		}
	}
	if (hasUnknown) {
		cout<<"Usage:"<<endl;
		cout<<"\t-noBorder \t Generate physical border \t Default: has border"<<endl;
		cout<<"\t-noBarrels \t Generate barrel  \t\t Default: can generate barrels"<<endl;
		cout<<"\t-noContainers \t Generate containers \t\t Default: can generate container"<<endl;
		cout<<"\t-items n \t Generate n items \t\t Default: 1"<<endl;
		return 0;
	}
	
	if (generateContainers == 0 && generateItems != 0 && generateBarrels == 0) {
		cerr<<endl;
		cerr<<"Error: you cannot have more than 0 items generated with both Containers and Barrels disabled!"<<endl;
		return 0;
	}

	string line;
	ifstream templateFile ("template.world");
	ofstream newWorldFile;
	
	if (templateFile.is_open()) {
		newWorldFile.open("generated.world");
		while(getline(templateFile, line)) {
			//cout << line << endl;
			newWorldFile << line << endl;
		}
		if (generateBorder) {
			newWorldFile<<"#phsical border" << endl;
			newWorldFile<<"border(color \"red\" pose [ 5 -3 0 0 ] size [10 0.1 0.5])" << endl;
			newWorldFile<<"border(color \"red\" pose [ 5 3 0 0 ]  size [10 0.1 0.5])" << endl;
			newWorldFile<<"border(color \"red\" pose [ 10 0 0 0 ] size [0.1 6 0.5])"  << endl;
			newWorldFile<<"border(color \"red\" pose [ 0 0 0 0 ]  size [0.1 6 0.5])"  << endl;
			cout<<"6m by 10m border generated (red)"<<endl;
		}else{
			newWorldFile<<"#floating border" << endl;
			newWorldFile<<"border(color \"grey\" pose [ 5 -3 1 0 ] size [10 0.1 0.5])" << endl;
			newWorldFile<<"border(color \"grey\" pose [ 5 3 1 0 ]  size [10 0.1 0.5])" << endl;
			newWorldFile<<"border(color \"grey\" pose [ 10 0 1 0 ] size [0.1 6 0.5])"  << endl;
			newWorldFile<<"border(color \"grey\" pose [ 0 0 1 0 ]  size [0.1 6 0.5])"  << endl;
			cout<<"6m by 10m border generated (grey) - these will not be seen by the robot"<<endl;
		}
		/*Gernate items*/
		int i = 0;
		srand(time(NULL)); // use current time as seed for random generator
		int itemToGenerate = 0; // 0 is barrel and 1 is container
		int itemX[generateItems+1];
		int itemY[generateItems+1];
		int itemR[generateItems+1];

		itemX[0] = 28;
		itemY[0] = 0;
		itemR[0] = 70;
		for (i = 0; i < generateItems; i++) {
			//border is from (0,-3) to (10,3)
			if (generateBarrels == 1 && generateContainers == 1) {
				itemToGenerate = (int)(3*(rand()/(RAND_MAX + 1.0)));
			}else if (generateBarrels) {
				itemToGenerate = 0;
			}else{
				itemToGenerate = 1;
			}


			if (itemToGenerate == 0) {
				cout<<i<<":Genearted barrel"<<endl;
				int x,y,r; //x,y raduis in cm
				r = 10+2*(int)(21*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 50cm
				
				char vaild = false;
				int tries = 0;
				while(vaild == false) {
					if (tries > 100) { //try too many times regenerate size
						r = 10+2*(int)(21*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 50cm
						tries = 0;
					}
					//x can be from 0 to 10 meters
					x = (int)(1001*(rand()/(RAND_MAX + 1.0)));
					//y can be from -3 to 3 meters
					y = (int)(601*(rand()/(RAND_MAX + 1.0)))-300;
					vaild = isVaild(x, y, r, i, itemX,itemY,itemR);

					tries++;
				}

				float xf,yf,rf; //floating point numbers in meters
				xf = (float)x/100.0;
				yf = (float)y/100.0;
				rf = (float)r/100.0;


				cout<<"Radius:"<<rf;
				cout<<" X:"<<xf;
				cout<<" Y:"<<yf<<endl;;	
				newWorldFile<<"circle(pose ["<<xf<<" "<<yf<<" 0 0] size ["<<rf<<" "<<rf<<" 0.500])"<<endl;
			}else{
				cout<<i<<":Genearted container"<<endl;
				int x,y,w,l; //x,y width and length in cm
				l = 10+2*(int)(46*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 90cm
				w = 10+2*(int)(46*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 90cm
				int r = sqrt((l/2)*(l/2)+(w/2)*(w/2));
				char vaild = false;
				int tries = 0;
				while(vaild == false) {
					if (tries > 100) { //try too many times regenerate size
						l = 10+2*(int)(46*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 90cm
						w = 10+2*(int)(46*(rand()/(RAND_MAX + 1.0))); //min 10 cm - increments of 2cm upto 90cm
						int r = sqrt(l*l+w*w);
						tries = 0;
					}
					//x can be from 0 to 10 meters
					x = (int)(1001*(rand()/(RAND_MAX + 1.0)));
					//y can be from -3 to 3 meters
					y = (int)(601*(rand()/(RAND_MAX + 1.0)))-300;
					vaild = isVaild(x, y, r, i, itemX,itemY,itemR);
					tries++;
				}

				int rotation; //in degrees
				rotation = (int)(180*(rand()/(RAND_MAX + 1.0))); //item can rotate from 0-179


				float xf,yf,lf,wf,rotationf; //floating point numbers in meters and radians
				xf = (float)x/100.0;
				yf = (float)y/100.0;
				lf = (float)l/100.0;
				wf = (float)w/100.0;

				cout<<"width:"<<w<< " length:"<<l<<" rotation:"<<rotation<<endl;
				cout<<" X:"<<xf;
				cout<<" Y:"<<yf<<endl;;	
				newWorldFile<<"rectangle(pose ["<<xf<<" "<<yf<<" 0 "<<rotation<<"] size ["<<lf<<" "<<wf<<" 0.500])"<<endl;
			}

			cout<<endl;
		}

		newWorldFile.close();
		templateFile.close();
	}else{
		cout<<"Unable to open template file"<<endl;
	}
	return 0;
	
}