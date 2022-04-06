

#include<stdio.h>
#include<stdlib.h>
//#include<math.h>
//#include<time.h>

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

#define UInitialColor 5
#define FInitialColor 1
#define LInitialColor 2
#define BInitialColor 3
#define RInitialColor 4
#define DInitialColor 0


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

                void RotCubePinceDroiteAntiHoraire();
                void RotCubePinceDroiteHoraire();
                void RotCubePinceGaucheAntiHoraire();
                void RotCubePinceGaucheHoraire();

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

                int WhereIsColor(int initialColorFace);

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

