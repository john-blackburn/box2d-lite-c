#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <float.h>

#include <gl/gl.h>
#include "b2d_lite.h"

#define ID_TIMER   1

extern int g_numArbiters;

int width = 256*3;      // width of the window in px
int height = 192*3;     // height of the window in px
float zoom = 10.0f;     // if pan_y=0, coord system goes from y=[-zoom,+zoom] (metres) and corresponding for x given aspect ratio
float pan_y =8.0f;      // pan_y shifts up and down. eg pan_y=zoom means y=0 is at bottom of screen. pan_y=0, y=0 at centre

struct Body ground, crate, crates[30];

struct Vec2 initVec2(float x, float y)
{
    struct Vec2 res;
    res.x=x;
    res.y=y;
    return res;
}

void DrawBody(struct Body* body)
{
	struct Mat22 R;
    initMat22(&R, body->rotation);
    
	struct Vec2 x = body->position;
	struct Vec2 h = scaledVec2(0.5f, body->width);

	struct Vec2 v1 = sumVec2(x, matmul(R, initVec2(-h.x, -h.y)));
	struct Vec2 v2 = sumVec2(x, matmul(R, initVec2( h.x, -h.y)));
	struct Vec2 v3 = sumVec2(x, matmul(R, initVec2( h.x,  h.y)));
	struct Vec2 v4 = sumVec2(x, matmul(R, initVec2(-h.x,  h.y)));

	glColor3f(0.8f, 0.8f, 0.9f);

	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

void EnableOpenGL(HWND hWnd, HDC * hDC, HGLRC * hRC)
{
  PIXELFORMATDESCRIPTOR pfd;
  int format,i;
	
  // get the device context (DC)
  *hDC = GetDC( hWnd );
	
  // set the pixel format for the DC
  ZeroMemory( &pfd, sizeof( pfd ) );
  pfd.nSize = sizeof( pfd );
  pfd.nVersion = 1;
  pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
  pfd.iPixelType = PFD_TYPE_RGBA;
  pfd.cColorBits = 24;
  pfd.cDepthBits = 16;
  pfd.iLayerType = PFD_MAIN_PLANE;
  format = ChoosePixelFormat( *hDC, &pfd );
  SetPixelFormat( *hDC, format, &pfd );
	
  // create and enable the render context (RC)
  *hRC = wglCreateContext( *hDC );
  wglMakeCurrent( *hDC, *hRC );

  glViewport(0, 0, width, height);
  
  float aspect = (float)width / (float)height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glOrtho(-zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
	
  glClearColor( 0.0f, 0.0f, 0.0f, 1.0f );
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void DisableOpenGL(HWND hWnd, HDC hDC, HGLRC hRC)
{
    wglMakeCurrent( NULL, NULL );
    wglDeleteContext( hRC );
    ReleaseDC( hWnd, hDC );
}

LRESULT CALLBACK WndProc (HWND hwnd, UINT iMsg, WPARAM wParam, LPARAM lParam)
{
    PAINTSTRUCT ps ;
    RECT        rect ;

    static HDC hDC;
    static HGLRC hRC;  
    static RECT clientRect,winRect;

    if (iMsg==WM_CREATE)
    {
        
        iterations = 10;
        gravity.x=0.0f;
        gravity.y=-10.0f;
        
        struct Vec2 size;
        size.x=20.f;            // ground is 20m x 2m
        size.y=2.0f;
        
        struct Vec2 pos;
        pos.x=0;
        pos.y=-0.5f * size.y;
        
        initBody(&ground, size, FLT_MAX);
        ground.position = pos;
        
        addBody(&ground);

        size.x=1.0f;        // each crate is a 1m x 1m square
        size.y=1.0f;

        for (int i = 0; i < 15; ++i)
        {
            initBody(&crates[i], size, 1.0f);
            crates[i].friction = 0.2f;

            float x = RandomRange(-0.1f, 0.1f);
            pos.x = x;
            pos.y = 0.51f + 1.05f * i;
            crates[i].position = pos;
            
            addBody(&crates[i]);
        }
        
        GetClientRect(hwnd,&clientRect);
        GetWindowRect(hwnd,&winRect);

        int dxChrome,dyChrome;
        
        printf("%d %d\n",winRect.bottom-winRect.top, clientRect.bottom - clientRect.top);

        dxChrome=winRect.right-winRect.left-(clientRect.right-clientRect.left);
        dyChrome=winRect.bottom-winRect.top-(clientRect.bottom-clientRect.top);

        SetWindowPos(hwnd,HWND_TOP,0,0,width+dxChrome,height+dyChrome,SWP_NOMOVE);

        EnableOpenGL(hwnd,&hDC,&hRC);

        return 0 ;
    }  
    else if (iMsg==WM_SIZE)
    {
        return 0;
    }
    else if (iMsg==WM_TIMER)
    {
        float dt = 1.0f/60.f;
        Step(dt);
        glClear(GL_COLOR_BUFFER_BIT);            
        
        DrawBody(&ground);        
        
        for (int i=0;i<15;i++)
            DrawBody(&crates[i]);
        
        SwapBuffers(hDC);
        
//        printf("ggg%d\n",g_numArbiters);

        return 0;
    }
    else if (iMsg==WM_CLOSE)
    {
        DisableOpenGL(hwnd,hDC,hRC);
        DestroyWindow(hwnd);
        return 0;
    }
    else if (iMsg==WM_DESTROY)
    {
        KillTimer(hwnd, ID_TIMER);
        PostQuitMessage (0) ;
        return 0 ;
    }

    return DefWindowProc(hwnd, iMsg, wParam, lParam) ;
}

int WINAPI WinMain (HINSTANCE hInstance, HINSTANCE hPrevInstance,
                    PSTR szCmdLine, int iCmdShow)
{
  static char szAppName[] = "b2dClass" ;
  HWND        hwnd ;
  MSG         msg ;
  WNDCLASSEX  wndclass ;
  
  wndclass.cbSize        = sizeof (wndclass) ;
  wndclass.style         = CS_HREDRAW | CS_VREDRAW ;
  wndclass.lpfnWndProc   = WndProc ;
  wndclass.cbClsExtra    = 0 ;
  wndclass.cbWndExtra    = 0 ;
  wndclass.hInstance     = hInstance ;
  wndclass.hIcon         = LoadIcon (GetModuleHandle(NULL), MAKEINTRESOURCE(999)) ;
  wndclass.hCursor       = LoadCursor (NULL, IDC_ARROW) ;
  wndclass.hbrBackground = (HBRUSH) GetStockObject (WHITE_BRUSH) ;
  wndclass.lpszMenuName  = MAKEINTRESOURCE(100);
  wndclass.lpszClassName = szAppName ;
  wndclass.hIconSm       = NULL ;
  
  RegisterClassEx (&wndclass) ;

  const char *name="b2d (OpenGL)";
  
  hwnd = CreateWindow (szAppName,         // window class name
		       name,     // window caption
		       WS_OVERLAPPED | WS_SYSMENU,     // window style
		       CW_USEDEFAULT,           // initial x position
		       CW_USEDEFAULT,           // initial y position
		       width,           // initial x size
		       height,           // initial y size
		       NULL,                    // parent window handle
		       NULL,                    // window menu handle
		       hInstance,               // program instance handle
		       NULL) ;		             // creation parameters


  SetTimer(hwnd, ID_TIMER, 0, NULL);  // 0 means run as fast as possible, in practice 60 fps

  // ----------------------------------------------------------------------
  // Create window
  // ----------------------------------------------------------------------

  printf("OpenGL version=%s\n",glGetString(GL_VERSION));

  ShowWindow (hwnd, iCmdShow) ;
  UpdateWindow (hwnd) ;

  while (GetMessage (&msg, NULL, 0, 0)) {
    TranslateMessage (&msg) ;
    DispatchMessage (&msg) ;
  }

  return msg.wParam ;
}
