

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<time.h>
#include<iostream>

#define WHITE 0
#define RED 1
#define GREEN 2
#define ORANGE 3
#define BLUE 4
#define YELLOW 5

#define U 0
#define F 1
#define L 2
#define B 3
#define R 4
#define D 5

using namespace std;


class RubiksCube
{
        public:

                /// constructeur
                RubiksCube();

                /// destructeur
                ~RubiksCube();

                void affichage() const;

                /// Amene la face voulu devant
                void BringToFront(int face);

                /// bring face to top
                void BringToTop(int face);

		/// bring top to Top and face to Face
		void BringToTop_Front(int top,int front);


                void MoveR();
                void MoveR2();
                void MoveRprime();

                void MoveL();
                void MoveL2();
                void MoveLprime();

                void MoveU();
                void MoveU2();
                void MoveUprime();

                void MoveD();
                void MoveD2(); 
                void MoveDprime();

                void MoveF();
                void MoveF2();
                void MoveFprime();

                void MoveB();
                void MoveB2();
                void MoveBprime();


        protected:

                /// cube data
                int **cube_data;

                /// temporary container for cube data
                int **temp_cube_data;

                int moves;

                /// copy temp_cube_data into cube_data
                void CopyCube();

                // building blocks for moving pieces
                void RotateFace(int);
                void RotateEquatorX();
                void RotateEquatorY();
                void RotateEquatorZ();
                void RotateRight();
                void RotateLeft();
                void RotateUp();
                void RotateDown();
                void RotateFront();
                void RotateBack();

};

