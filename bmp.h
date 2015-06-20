/*Load bitmap file */
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glaux.lib")


#include <windows.h>        
#include <gl\gl.h>           
#include <gl\glu.h>            
#include <gl\glaux.h>       

bool loadbmp(UINT textureArray[], LPSTR strFileName, int ID)
{
     if(!strFileName)   return false;
     AUX_RGBImageRec *pBitMap = auxDIBImageLoad(strFileName); 
   
     if(pBitMap == NULL)    exit(0);
     glGenTextures(1, &textureArray[ID]);

     glBindTexture(GL_TEXTURE_2D, textureArray[ID]);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
     glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
     //create our image  
     glTexImage2D(GL_TEXTURE_2D, 0, 3, pBitMap->sizeX, pBitMap->sizeY, 0, GL_RGB, GL_UNSIGNED_BYTE, pBitMap->data);

     //Clear value in pBitMap
     if (pBitMap)                                       
     {
        if (pBitMap->data)                                
        {
            delete (pBitMap->data);                        
        }
        delete (pBitMap);                                    
     }
     return true;
}
