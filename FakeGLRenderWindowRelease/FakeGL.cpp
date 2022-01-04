//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5812M Foundations of Modelling & Rendering
//  User Interface for Coursework
//
//  September, 2020
//
//  ------------------------
//  FakeGL.cpp
//  ------------------------
//  
//  A unit for implementing OpenGL workalike calls
//  
///////////////////////////////////////////////////

#include "FakeGL.h"
#include <math.h>
#include <cmath>
#include <limits>

#define PI 3.14159265

//-------------------------------------------------//
//                                                 //
// CONSTRUCTOR / DESTRUCTOR                        //
//                                                 //
//-------------------------------------------------//

// constructor
FakeGL::FakeGL()
    { // constructor
        // Initialise view matrix
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                if(col == row)
                    viewMatrix.coordinates[row][col] = 1;
                else
                    viewMatrix.coordinates[row][col] = 0;
            }
        }

        // Initialise projection matrix
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                projectionMatrix.coordinates[row][col] = 0;
            }
        }

        // Initialise the value for the light position
        lightVector.x = 0;
        lightVector.y = 0;
        lightVector.z = 0;
        lightVector.w = 0;

        // Initialise the state value for normal
        normalX = 0.0;
        normalY = 0.0;
        normalZ = 1.0;
    } // constructor

// destructor
FakeGL::~FakeGL()
    { // destructor
        //for()
    } // destructor

//-------------------------------------------------//
//                                                 //
// GEOMETRIC PRIMITIVE ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// starts a sequence of geometric primitives
void FakeGL::Begin(unsigned int PrimitiveType)
    { // Begin()
        // Initialise primitives state
        // It is initially set to -1 
        PrimitiveState = PrimitiveType;
    } // Begin()

// ends a sequence of geometric primitives
void FakeGL::End()
    { // End()
        TransformVertex();
        RasterisePrimitive();
        ProcessFragment();
        // Reset the primitive state
        PrimitiveState = -1;
    } // End()

// sets the size of a point for drawing
void FakeGL::PointSize(float size)
    { // PointSize()
        pointSize = size;
    } // PointSize()

// sets the width of a line for drawing purposes
void FakeGL::LineWidth(float width)
    { // LineWidth()
        lineWidth = width;
    } // LineWidth()

//-------------------------------------------------//
//                                                 //
// MATRIX MANIPULATION ROUTINES                    //
//                                                 //
//-------------------------------------------------//

// set the matrix mode (i.e. which one we change)   
void FakeGL::MatrixMode(unsigned int whichMatrix)
    { // MatrixMode()
        matrixModeMatrix = whichMatrix;
    } // MatrixMode()

// pushes a matrix on the stack
void FakeGL::PushMatrix()
    { // PushMatrix()
        if(matrixModeMatrix == FAKEGL_MODELVIEW)
        {
            modelViewMatrices.push(currentMatrix);
        }

        if(matrixModeMatrix == FAKEGL_PROJECTION)
        {
            projectionMatrices.push(currentMatrix);
        }
    } // PushMatrix()

// pops a matrix off the stack
void FakeGL::PopMatrix()
    { // PopMatrix()
        if(matrixModeMatrix == FAKEGL_MODELVIEW)
        {
            modelViewMatrices.pop();
        }

        if(matrixModeMatrix == FAKEGL_PROJECTION)
        {
            projectionMatrices.pop();
        }
    } // PopMatrix()

// load the identity matrix
void FakeGL::LoadIdentity()
    { // LoadIdentity()
        if(matrixModeMatrix == FAKEGL_MODELVIEW)
        {
            matrixModeMatrices = modelViewMatrices;
        }

        if(matrixModeMatrix == FAKEGL_PROJECTION)
        {
            matrixModeMatrices = projectionMatrices;
        }
    } // LoadIdentity()

// multiply by a known matrix in column-major format
void FakeGL::MultMatrixf(const float *columnMajorCoordinates)
    { // MultMatrixf()
        // Creating the matrix by coping values from the parameter
        Matrix4 matrix;
        int i = 0;
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                matrix[row][col] = columnMajorCoordinates[i];
                i++;
            }
        }
        matrixModeMatrices.push(matrix);
    } // MultMatrixf()

// sets up a perspective projection matrix
void FakeGL::Frustum(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Frustum()
        Matrix4 frustumMatrix;
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                frustumMatrix[row][col] = 0;
            }
        }

        // Frustum matrix calculations
        frustumMatrix[0][0] = (2 * zNear) / (right - left);
        frustumMatrix[0][2] = (right + left) / (right - left);

        frustumMatrix[1][1] = (2 * zNear) / (top - bottom);
        frustumMatrix[1][2] = (top + bottom) / (top - bottom);

        frustumMatrix[2][2] = -(zFar + zNear) / (zFar - zNear);
        frustumMatrix[2][3] = -(2 * zFar * zNear) / (zFar - zNear);

        frustumMatrix[3][2] = -1;

        // Multiply the frustum matrix to the current matrix
        currentMatrix = frustumMatrix * currentMatrix;
    } // Frustum()

// sets an orthographic projection matrix
void FakeGL::Ortho(float left, float right, float bottom, float top, float zNear, float zFar)
    { // Ortho()
        this->zNear = zNear;
        this->zFar = zFar;
        // Resetting the projection matrix
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                projectionMatrix.coordinates[row][col] = 0;
            }
        }

        // Frustum ortho calculations
        // Change -0 to 0 to prevent -0 affecting the value
        projectionMatrix[0][0] = 2/(right-left);
        if(-((right+left)/(right-left)) == -0)
            projectionMatrix[0][3] = 0;
        else
            projectionMatrix[0][3] = -((right+left)/(right-left));

        projectionMatrix[1][1] = 2/(top-bottom);
        if(-((top+bottom)/(top-bottom)) == -0)
            projectionMatrix[1][3] = 0;
        else
            projectionMatrix[1][3] = -((top+bottom)/(top-bottom));

        projectionMatrix[2][2] = -(2/(zFar-zNear));
        if(-((zFar+zNear)/(zFar-zNear)) == -0)
            projectionMatrix[2][3] = 0;
        else
            projectionMatrix[2][3] = -((zFar+zNear)/(zFar-zNear));

        projectionMatrix[3][3] = 1;

        //std::cout << "Matrix: " << "\n";
        //std::cout << projectionMatrix << "\n";
    } // Ortho()

// rotate the matrix
void FakeGL::Rotatef(float angle, float axisX, float axisY, float axisZ)
    { // Rotatef()
        std::cout << "rotate" << "\n";

        // Initialise three rotation matrices
        Matrix4 xRotationMatrix;
        Matrix4 yRotationMatrix;
        Matrix4 zRotationMatrix;
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                if(row == col)
                {
                    xRotationMatrix[row][col] = 1;
                    yRotationMatrix[row][col] = 1;
                    zRotationMatrix[row][col] = 1;
                }
                else
                {
                    xRotationMatrix[row][col] = 0;
                    yRotationMatrix[row][col] = 0;
                    zRotationMatrix[row][col] = 0;
                }
            }
        }

        // Convert degree to radian as c++ cos and sin takes a radian parameter
        float radianValue = (angle * PI) /180.0;

        // X rotation matrix
        if(axisX == 1)
        {
            xRotationMatrix[1][1] = cos(radianValue);
            xRotationMatrix[1][2] = -sin(radianValue);
            xRotationMatrix[2][1] = sin(radianValue);
            xRotationMatrix[2][2] = cos(radianValue);
        }

        // Y rotation matrix
        if(axisY == 1)
        {
            yRotationMatrix[0][0] = cos(radianValue);
            yRotationMatrix[0][2] = sin(radianValue);
            yRotationMatrix[2][0] = -sin(radianValue);
            yRotationMatrix[2][2] = cos(radianValue);
        }

        // Z rotation matrix
        if(axisZ == 1)
        {
            zRotationMatrix[0][0] = cos(radianValue);
            zRotationMatrix[0][1] = -sin(radianValue);
            zRotationMatrix[1][0] = sin(radianValue);
            zRotationMatrix[1][1] = cos(radianValue);
        }

        // Adding up the matrices
        Matrix4 rotationMatrix = zRotationMatrix * yRotationMatrix * xRotationMatrix;

        // Pushing the matrix to matrix mode matrices for later calculations
        matrixModeMatrices.push(rotationMatrix);
    } // Rotatef()

// scale the matrix
void FakeGL::Scalef(float xScale, float yScale, float zScale)
    { // Scalef()
        // Initialise the scaling matrix
        Matrix4 scalingMatrix;
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                scalingMatrix[row][col] = 0;
            }
        }

        scalingMatrix[0][0] = xScale;
        scalingMatrix[1][1] = yScale;
        scalingMatrix[2][2] = zScale;
        scalingMatrix[3][3] = 1;

        // Pushing the matrix to matrix mode matrices for later calculations
        matrixModeMatrices.push(scalingMatrix);
    } // Scalef()

// translate the matrix
void FakeGL::Translatef(float xTranslate, float yTranslate, float zTranslate)
    { // Translatef()
        // Intialise the translation matrix
        Matrix4 translationMatrix;
        for(int col = 0; col < 4; col++)
        {
            for(int row = 0; row < 4; row++)
            {
                if(col == row)
                    translationMatrix[row][col] = 1;
                else
                    translationMatrix[row][col] = 0;
            }
        }

        translationMatrix[0][3] = xTranslate;
        translationMatrix[1][3] = yTranslate;
        translationMatrix[2][3] = zTranslate;

        // Pushing the matrix to matrix mode matrices for later calculations
        matrixModeMatrices.push(translationMatrix);
        
    } // Translatef()

// sets the viewport
void FakeGL::Viewport(int x, int y, int width, int height)
    { // Viewport()
        screenWidth = width;
        screenHeight = height;
    } // Viewport()

//-------------------------------------------------//
//                                                 //
// VERTEX ATTRIBUTE ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// sets colour with floating point
void FakeGL::Color3f(float red, float green, float blue)
    { // Color3f()
        // RGBA Color State
        // Multiple the color by 255 here to prevent confusing later
        colorRed = red * 255;
        colorGreen = green * 255;
        colorBlue = blue * 255;
        colorAlpha = 255;
    } // Color3f()

// sets material properties
void FakeGL::Materialf(unsigned int parameterName, const float parameterValue)
    { // Materialf()
        if(parameterName == FAKEGL_SHININESS)
        {
            specularCoefficientMaterial = parameterValue;
        }
    } // Materialf()

void FakeGL::Materialfv(unsigned int parameterName, const float *parameterValues)
    { // Materialfv()
        // Setting the material state for emission, ambient, diffuse and specular
        if(parameterName == FAKEGL_EMISSION)
        {
            emissiveColour[0] = parameterValues[0];
            emissiveColour[1] = parameterValues[1];
            emissiveColour[2] = parameterValues[2];
            emissiveColour[3] = parameterValues[3];
        }
        if(parameterName == FAKEGL_AMBIENT_AND_DIFFUSE)
        {
            ambientAndDiffuseMaterial[0] = parameterValues[0];
            ambientAndDiffuseMaterial[1] = parameterValues[1];
            ambientAndDiffuseMaterial[2] = parameterValues[2];
            ambientAndDiffuseMaterial[3] = parameterValues[3];
        }
        if(parameterName == FAKEGL_SPECULAR)
        {
            specularMaterial[0] = parameterValues[0];
            specularMaterial[1] = parameterValues[1];
            specularMaterial[2] = parameterValues[2];
            specularMaterial[3] = parameterValues[3];
        }
    } // Materialfv()

// sets the normal vector
void FakeGL::Normal3f(float x, float y, float z)
    { // Normal3f()
        // Setting the state for three normal axis
        normalX = x;
        normalY = y;
        normalZ = z;
    } // Normal3f()

// sets the texture coordinates
void FakeGL::TexCoord2f(float u, float v)
    { // TexCoord2f()
        // Setting the vertex texture coordinate
        textureCoordU = u * textureBuffer.width;
        textureCoordV = v * textureBuffer.height;
    } // TexCoord2f()

// sets the vertex & launches it down the pipeline
void FakeGL::Vertex3f(float x, float y, float z)
    { // Vertex3f()
        // Create a vertex and copying all values from the current state value
        vertexWithAttributes v;
        v.position.x = x;
        v.position.y = y;
        v.position.z = z;
        v.position.w = 1;

        v.colour.red = colorRed;
        v.colour.green = colorGreen;
        v.colour.blue = colorBlue;
        v.colour.alpha = colorAlpha;

        v.textureCoordX = textureCoordU;
        v.textureCoordY = textureCoordV;

        v.normal.x = normalX;
        v.normal.y = normalY;
        v.normal.z = normalZ;

        v.ambientReflectivity[0] = ambientAndDiffuseMaterial[0];
        v.ambientReflectivity[1] = ambientAndDiffuseMaterial[1];
        v.ambientReflectivity[2] = ambientAndDiffuseMaterial[2];
        v.ambientReflectivity[3] = ambientAndDiffuseMaterial[3];
        
        v.diffuseReflectivity[0] = ambientAndDiffuseMaterial[0];
        v.diffuseReflectivity[1] = ambientAndDiffuseMaterial[1];
        v.diffuseReflectivity[2] = ambientAndDiffuseMaterial[2];
        v.diffuseReflectivity[3] = ambientAndDiffuseMaterial[3];

        v.specularReflectivity[0] = specularMaterial[0];
        v.specularReflectivity[1] = specularMaterial[1];
        v.specularReflectivity[2] = specularMaterial[2];
        v.specularReflectivity[3] = specularMaterial[3];

        v.specularCoefficient = specularCoefficientMaterial;

        // Push the current vertex to the vertex queue
        vertexQueue.push_back(v);
    } // Vertex3f()

//-------------------------------------------------//
//                                                 //
// STATE VARIABLE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// disables a specific flag in the library
void FakeGL::Disable(unsigned int property)
    { // Disable()
        if(property == FAKEGL_LIGHTING)
            lightingEnable = false;
        if(property == FAKEGL_TEXTURE_2D)
            texture2DEnable = false;
        if(property == FAKEGL_DEPTH_TEST)
            depthTestEnable = false;
        if(property == FAKEGL_PHONG_SHADING)
            phongShadingEnable = false;
    } // Disable()

// enables a specific flag in the library
void FakeGL::Enable(unsigned int property)
    { // Enable()
        if(property == FAKEGL_LIGHTING)
            lightingEnable = true;
        if(property == FAKEGL_TEXTURE_2D)
            texture2DEnable = true;
        if(property == FAKEGL_DEPTH_TEST)
            depthTestEnable = true;
        if(property == FAKEGL_PHONG_SHADING)
            phongShadingEnable = true;
    } // Enable()

//-------------------------------------------------//
//                                                 //
// LIGHTING STATE ROUTINES                         //
//                                                 //
//-------------------------------------------------//

// sets properties for the one and only light
void FakeGL::Light(int parameterName, const float *parameterValues)
    { // Light()
        // Setting the light position
        // And apply rotation, translation if needing
        if(parameterName == FAKEGL_POSITION)
        {
            lightVector.x = parameterValues[0];
            lightVector.y = parameterValues[1];
            lightVector.z = parameterValues[2];
            lightVector.w = parameterValues[3];

            lightVector = viewMatrix * lightVector;

            std::stack<Matrix4> stackCopy = matrixModeMatrices;

            if(!stackCopy.empty())
            {
                while(!stackCopy.empty())
                {
                    lightVector = stackCopy.top() * lightVector;
                    stackCopy.pop();
                }
            }
        }

        // Setting the ambient, diffuse and specular colour of the light
        if(parameterName == FAKEGL_AMBIENT)
        {
            ambientColour[0] = parameterValues[0];
            ambientColour[1] = parameterValues[1];
            ambientColour[2] = parameterValues[2];
            ambientColour[3] = parameterValues[3];
        }
        if(parameterName == FAKEGL_DIFFUSE)
        {
            diffuseColour[0] = parameterValues[0];
            diffuseColour[1] = parameterValues[1];
            diffuseColour[2] = parameterValues[2];
            diffuseColour[3] = parameterValues[3];
        }
        if(parameterName == FAKEGL_SPECULAR)
        {
            specularColour[0] = parameterValues[0];
            specularColour[1] = parameterValues[1];
            specularColour[2] = parameterValues[2];
            specularColour[3] = parameterValues[3];
        }
    } // Light()

//-------------------------------------------------//
//                                                 //
// TEXTURE PROCESSING ROUTINES                     //
//                                                 //
// Note that we only allow one texture             //
// so glGenTexture & glBindTexture aren't needed   //
//                                                 //
//-------------------------------------------------//

// sets whether textures replace or modulate
void FakeGL::TexEnvMode(unsigned int textureMode)
    { // TexEnvMode()
        fakeTextureMode = textureMode;
    } // TexEnvMode()

// sets the texture image that corresponds to a given ID
void FakeGL::TexImage2D(const RGBAImage &textureImage)
    { // TexImage2D()
        //textureBuffer.Resize(textureImage.width, textureImage.height);
        textureBuffer = textureImage;
    } // TexImage2D()

//-------------------------------------------------//
//                                                 //
// FRAME BUFFER ROUTINES                           //
//                                                 //
//-------------------------------------------------//

// clears the frame buffer
void FakeGL::Clear(unsigned int mask)
    { // Clear()
        if(mask == FAKEGL_COLOR_BUFFER_BIT)
        {
            for(int i = 0; i < frameBuffer.width; i++)
            {
                for(int j = 0; j < frameBuffer.height; j++)
                {
                    frameBuffer[j][i].red = clearColorRed * 255;
                    frameBuffer[j][i].green = clearColorGreen * 255;
                    frameBuffer[j][i].blue = clearColorBlue * 255;
                    frameBuffer[j][i].alpha = clearColorAlpha * 255;
                }
            }
        }

        if(mask == FAKEGL_DEPTH_BUFFER_BIT)
        {
            depthBuffer.Resize(screenWidth, screenHeight);
            for(int j = 0; j < depthBuffer.height; j++)
            {
                for(int i = 0; i < depthBuffer.width; i++)
                {
                    depthBuffer[j][i].alpha = std::numeric_limits<float>::infinity();
                }
            }
        }

        // Run clear color and clear depth buffer at the same time
        // Since some part of the code add two mask values together
        if(mask == (FAKEGL_COLOR_BUFFER_BIT + FAKEGL_DEPTH_BUFFER_BIT))
        {
            //Clear Color
            for(int i = 0; i < frameBuffer.width; i++)
            {
                for(int j = 0; j < frameBuffer.height; j++)
                {
                    frameBuffer[j][i].red = clearColorRed * 255;
                    frameBuffer[j][i].green = clearColorGreen * 255;
                    frameBuffer[j][i].blue = clearColorBlue * 255;
                    frameBuffer[j][i].alpha = clearColorAlpha * 255;
                }
            }

            // Clear depth buffer
            depthBuffer.Resize(screenWidth, screenHeight);
            for(int j = 0; j < depthBuffer.height; j++)
            {
                for(int i = 0; i < depthBuffer.width; i++)
                {
                    depthBuffer[j][i].alpha = std::numeric_limits<float>::infinity();
                }
            }
        }
    } // Clear()

// sets the clear colour for the frame buffer
void FakeGL::ClearColor(float red, float green, float blue, float alpha)
    { // ClearColor()
        clearColorRed = red;
        clearColorGreen = green;
        clearColorBlue = blue;
        clearColorAlpha = alpha;
    } // ClearColor()

//-------------------------------------------------//
//                                                 //
// MAJOR PROCESSING ROUTINES                       //
//                                                 //
//-------------------------------------------------//

// transform one vertex & shift to the raster queue
void FakeGL::TransformVertex()
    { // TransformVertex()
        // Without Matrix transformation
        /*
        for(vertexWithAttributes v:vertexQueue)
        {
            std::cout << "Position: " << v.position << "\n";
            std::cout << "Color: " << v.colour << "\n";

            screenVertexWithAttributes ndcs;
            ndcs.position.x = 256;
            ndcs.position.y = 256;
            ndcs.position.z = 0;

            ndcs.colour.red = 1.0;
            ndcs.colour.alpha = 1.0;

            rasterQueue.push_back(ndcs);
        }
        */

        for(vertexWithAttributes v:vertexQueue)
        {

            // Convert to OCS
            vertexWithAttributes ocs;
            ocs = v;

            // Convert to WCS
            vertexWithAttributes wcs;
            wcs = ocs;

            // Convert to VCS
            vertexWithAttributes vcs;
            vcs = wcs;
            vcs.position = viewMatrix * vcs.position;
            vcs.normal = viewMatrix * vcs.normal;

            // Convert to CCS
            vertexWithAttributes ccs;
            ccs = vcs;
            std::stack<Matrix4> stackCopy = matrixModeMatrices;

            // apply all transformation vertices on the vertex position and normal
            if(!stackCopy.empty())
            {
                while(!stackCopy.empty())
                {
                    ccs.position = stackCopy.top() * ccs.position;
                    ccs.normal = stackCopy.top() * ccs.normal;
                    stackCopy.pop();
                }
            }

            // Keep a copy of ccs for light calculations later
            vertexWithAttributes ccsCopy = ccs;
            
            ccs.position = projectionMatrix * ccs.position;
            ccs.normal = projectionMatrix * ccs.normal;

            // Convert to NDCS
            vertexWithAttributes ndcs = ccs;
            ndcs.position.x = ndcs.position.x / ndcs.position.w;
            ndcs.position.y = ndcs.position.y / ndcs.position.w;
            ndcs.position.z = ndcs.position.z / ndcs.position.w;
            ndcs.position.w = 1;

            // Convert to DCS
            int size_width = screenWidth/2;
            int size_height = screenHeight/2;

            screenVertexWithAttributes dcs;
            // DCS calculations
            dcs.position.x = (size_width * ndcs.position.x) + size_width;
            dcs.position.y = (size_height * (1) * ndcs.position.y) + size_height;
            dcs.position.z = ndcs.position.z;

            dcs.normal.x = ndcs.normal.x;
            dcs.normal.y = ndcs.normal.y;
            dcs.normal.z = -ndcs.normal.z;
            
            // If lighting is enabled do lighting calculations
            // Otherwise, inherit the colour from the vertex
            if(lightingEnable)
            {   
                for(int i = 0; i < 4; i++)
                {
                    // Ambient
                    dcs.ambientReflectivity[i]
                        = v.ambientReflectivity[i];

                    // Diffuse
                    dcs.diffuseReflectivity[i]
                        = v.diffuseReflectivity[i];

                    // Specular
                    dcs.specularReflectivity[i]
                        = v.specularReflectivity[i];
                }

                // Specular Coefficient
                dcs.specularCoefficient = v.specularCoefficient;

                // Ambient Light calculations
                float ambientResult[4];
                ambientResult[0] = ambientColour[0] * v.ambientReflectivity[0];
                ambientResult[1] = ambientColour[1] * v.ambientReflectivity[1];
                ambientResult[2] = ambientColour[2] * v.ambientReflectivity[2];
                ambientResult[3] = ambientColour[3] * v.ambientReflectivity[3];

                // Diffuse Light Calculations
                float diffuseResult[4];

                // Calculate cosine theta first
                double cosTheta = 0.0;
                Cartesian3 lightDirection;
                if(lightVector.w > 0)
                {
                    Cartesian3 vectorToLight = lightVector.Vector() - ccsCopy.position.Point();
                    lightDirection = vectorToLight.unit();
                }
                else
                {
                    lightDirection = lightVector.Vector();
                }             

                Cartesian3 normalisedNormal = ccsCopy.normal.unit();

                float normalNorm = ccsCopy.normal.unit().length();
                float lightDirectionNorm = lightDirection.length();

                float diffuseDotProduct = normalisedNormal.dot(lightDirection);

                // Check for negative dot product to prevent
                // the object from absorbing light
                if(diffuseDotProduct < 0)
                    diffuseDotProduct = 0;

                cosTheta = diffuseDotProduct / (normalNorm * lightDirectionNorm);

                diffuseResult[0] = diffuseColour[0] * v.diffuseReflectivity[0] * cosTheta;
                diffuseResult[1] = diffuseColour[1] * v.diffuseReflectivity[1] * cosTheta;
                diffuseResult[2] = diffuseColour[2] * v.diffuseReflectivity[2] * cosTheta;
                diffuseResult[3] = diffuseColour[3] * v.diffuseReflectivity[3] * cosTheta;


                // Specular Reflection calculations
                float specularResult[4];

                double cosThetaSpecular = 0.0;
                // Vector from vertex to light has already been calculated before

                // Eye coordinate
                Cartesian3 eyeVector(0, 0, 10);

                Cartesian3 vectorToEye = eyeVector - ccsCopy.position.Point();
                Cartesian3 eyeDirection = vectorToEye.unit();

                Cartesian3 bisector = (lightDirection + eyeDirection)/2;
                //Cartesian3 normalisedBisector = bisector.unit();

                dcs.fragLightVector = lightDirection;
                dcs.fragEyeVector = vectorToEye;

                // Normalised normal has been calculated
                //Cartesian3 normalisedNormalSpecular = ccsCopy.normal.unit();

                float bisectorNorm = bisector.length();

                float specularDotProduct = normalisedNormal.dot(bisector);

                if(specularDotProduct < 0)
                    specularDotProduct = 0;

                cosThetaSpecular = specularDotProduct
                                    / (normalNorm * bisectorNorm);

                specularResult[0] = specularColour[0] 
                                * v.specularReflectivity[0] 
                                * std::pow(cosThetaSpecular, v.specularCoefficient);
                
                specularResult[1] = specularColour[1] 
                                * v.specularReflectivity[1] 
                                * std::pow(cosThetaSpecular, v.specularCoefficient);
                
                specularResult[2] = specularColour[2] 
                                * v.specularReflectivity[2] 
                                * std::pow(cosThetaSpecular, v.specularCoefficient);
                
                specularResult[3] = specularColour[3] 
                                * v.specularReflectivity[3] 
                                * std::pow(cosThetaSpecular, v.specularCoefficient);
                
                // Adding all values from emissive, ambient, diffuse and specular together
                float allResult[4];
                allResult[0] = (emissiveColour[0] + ambientResult[0] 
                    + diffuseResult[0] + specularResult[0]);
                allResult[1] = (emissiveColour[1] + ambientResult[1] 
                    + diffuseResult[1] + specularResult[1]);
                allResult[2] = (emissiveColour[2] + ambientResult[2] 
                    + diffuseResult[2] + specularResult[2]);
                allResult[3] = (emissiveColour[3] + ambientResult[3] 
                    + diffuseResult[3] + specularResult[3]);

                // Clamp the values for RGBA channel
                if(allResult[0] < 0)
                    dcs.colour.red = 0;
                else if(allResult[0] > 1)
                    dcs.colour.red = 255;
                else
                    dcs.colour.red = allResult[0] * 255;

                if(allResult[1] < 0)
                    dcs.colour.green = 0;
                else if(allResult[1] > 1)
                    dcs.colour.green = 255;
                else
                    dcs.colour.green = allResult[1] * 255;

                if(allResult[2] < 0)
                    dcs.colour.blue = 0;
                else if(allResult[2] > 1)
                    dcs.colour.blue = 255;
                else
                    dcs.colour.blue = allResult[2] * 255;

                if(allResult[3] < 0)
                    dcs.colour.alpha = 0;
                else if(allResult[3] > 1)
                    dcs.colour.alpha = 255;
                else
                    dcs.colour.alpha = allResult[3] * 255;
            }
            else
            {
                dcs.colour.red = v.colour.red;
                dcs.colour.green = v.colour.green;
                dcs.colour.blue = v.colour.blue;
                dcs.colour.alpha = v.colour.alpha;
            }

            // Inherit the texture coordinate data as well
            dcs.textureCoordX = v.textureCoordX;
            dcs.textureCoordY = v.textureCoordY;

            // Push dcs to the raster queue
            rasterQueue.push_back(dcs);
        }

        // Clear the vertex queue after use
        vertexQueue.clear();

    } // TransformVertex()

// rasterise a single primitive if there are enough vertices on the queue
bool FakeGL::RasterisePrimitive()
    { // RasterisePrimitive()
        if(rasterQueue.empty())
            return false;
        else
            // Run different rasterise functions based on the current primitive state
            if(PrimitiveState == FAKEGL_POINTS)
            {
                for(screenVertexWithAttributes r:rasterQueue)
                {
                    RasterisePoint(r);
                    rasterQueue.pop_front();
                }
            }
            if(PrimitiveState == FAKEGL_LINES)
            {
                for(unsigned int i = 0; i < rasterQueue.size(); i+=2)
                {
                    if(i != rasterQueue.size()-1)
                    {
                        RasteriseLineSegment(rasterQueue[i], rasterQueue[i+1]);
                    }
                }
            }
            if(PrimitiveState == FAKEGL_TRIANGLES)
            {
                for(unsigned int i = 0; i < rasterQueue.size(); i+=3)
                {
                    if(i != rasterQueue.size()-2)
                    {
                        RasteriseTriangle(rasterQueue[i], rasterQueue[i+1], rasterQueue[i+2]);
                    }
                    //rasterQueue.pop_front();
                }
            }
            // Clear the raster Queue after use
            rasterQueue.clear();
            return true;
        
    } // RasterisePrimitive()

// rasterises a single point
void FakeGL::RasterisePoint(screenVertexWithAttributes &vertex0)
    { // RasterisePoint()

        // Set the size of the point we want to draw
        PointSize(lineWidth);

        // Increase the size of the point by creating more vertex
        for(int i = -pointSize; i < pointSize; i++)
        {
            for(int j = -pointSize; j < pointSize; j++)
            {
                if(vertex0.position.y + j < frameBuffer.height
                    && vertex0.position.x + i < frameBuffer.width)
                {
                    fragmentWithAttributes f;
                
                    f.row = vertex0.position.y + j;
                    f.col = vertex0.position.x + i;

                    f.colour.red = vertex0.colour.red;
                    f.colour.green = vertex0.colour.green;
                    f.colour.blue = vertex0.colour.blue;
                    f.colour.alpha = vertex0.colour.alpha;
                    
                    // Depth Test if enabled
                    if(depthTestEnable)
                    {   
                        // Update the value of the depth
                        if(vertex0.position.z < depthBuffer[f.row][f.col].alpha)
                        {   
                            // Make sure we are not rendering outside of the screen
                            if(f.row <= frameBuffer.height &&
                                f.row >= 0 && 
                                f.col <= frameBuffer.width &&
                                f.col >= 0)
                            {
                                depthBuffer[f.row][f.col].alpha = vertex0.position.z;
                                fragmentQueue.push_back(f);
                            }
                        }
                    }
                    else
                    {
                        // Make sure we are not rendering outside of the screen
                        if(f.row <= frameBuffer.height &&
                            f.row >= 0 && 
                            f.col <= frameBuffer.width &&
                            f.col >= 0)
                            fragmentQueue.push_back(f);
                    }
                }
            }
        }
    } // RasterisePoint()

// rasterises a single line segment
void FakeGL::RasteriseLineSegment(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1)
    { // RasteriseLineSegment()
        RasterisePoint(vertex0);
        RasterisePoint(vertex1);

        if((vertex0.position.z > zNear) && (vertex0.position.z < zFar)
            && (vertex1.position.z > zNear) && (vertex1.position.z < zFar))
        {
            // Create a line by rasterising point for a certain amount
            // Here we rastering a point every 0.005
            for(float t = 0.0; t <= 1.0; t+=0.005)
            {
                float point_x = vertex0.position.x + (vertex1.position.x-vertex0.position.x) * t;
                float point_y = vertex0.position.y + (vertex1.position.y-vertex0.position.y) * t;
                float point_z = vertex0.position.z + (vertex1.position.z-vertex0.position.z) * t;

                screenVertexWithAttributes v;
                v.position.x = point_x;
                v.position.y = point_y;
                v.position.z = point_z;

                v.colour.red = (float)vertex0.colour.red + ((float)vertex1.colour.red - (float)vertex0.colour.red) * t;
                v.colour.green = (float)vertex0.colour.green + ((float)vertex1.colour.green - (float)vertex0.colour.green) * t;
                v.colour.blue = (float)vertex0.colour.blue + ((float)vertex1.colour.blue - (float)vertex0.colour.blue) * t;
                v.colour.alpha = (float)vertex0.colour.alpha + ((float)vertex1.colour.alpha - (float)vertex0.colour.alpha) * t;

                RasterisePoint(v);
            }   
        }

    } // RasteriseLineSegment()

// rasterises a single triangle
void FakeGL::RasteriseTriangle(screenVertexWithAttributes &vertex0, screenVertexWithAttributes &vertex1, screenVertexWithAttributes &vertex2)
    { // RasteriseTriangle()

    // compute a bounding box that starts inverted to frame size
    // clipping will happen in the raster loop proper
    float minX = frameBuffer.width, maxX = 0.0;
    float minY = frameBuffer.height, maxY = 0.0;
    
    // test against all vertices
    if (vertex0.position.x < minX) minX = vertex0.position.x;
    if (vertex0.position.x > maxX) maxX = vertex0.position.x;
    if (vertex0.position.y < minY) minY = vertex0.position.y;
    if (vertex0.position.y > maxY) maxY = vertex0.position.y;
    
    if (vertex1.position.x < minX) minX = vertex1.position.x;
    if (vertex1.position.x > maxX) maxX = vertex1.position.x;
    if (vertex1.position.y < minY) minY = vertex1.position.y;
    if (vertex1.position.y > maxY) maxY = vertex1.position.y;
    
    if (vertex2.position.x < minX) minX = vertex2.position.x;
    if (vertex2.position.x > maxX) maxX = vertex2.position.x;
    if (vertex2.position.y < minY) minY = vertex2.position.y;
    if (vertex2.position.y > maxY) maxY = vertex2.position.y;

    // now for each side of the triangle, compute the line vectors
    Cartesian3 vector01 = vertex1.position - vertex0.position;
    Cartesian3 vector12 = vertex2.position - vertex1.position;
    Cartesian3 vector20 = vertex0.position - vertex2.position;

    // now compute the line normal vectors
    Cartesian3 normal01(-vector01.y, vector01.x, 0.0);  
    Cartesian3 normal12(-vector12.y, vector12.x, 0.0);  
    Cartesian3 normal20(-vector20.y, vector20.x, 0.0);  

    // we don't need to normalise them, because the square roots will cancel out in the barycentric coordinates
    float lineConstant01 = normal01.dot(vertex0.position);
    float lineConstant12 = normal12.dot(vertex1.position);
    float lineConstant20 = normal20.dot(vertex2.position);

    // and compute the distance of each vertex from the opposing side
    float distance0 = normal12.dot(vertex0.position) - lineConstant12;
    float distance1 = normal20.dot(vertex1.position) - lineConstant20;
    float distance2 = normal01.dot(vertex2.position) - lineConstant01;

    // if any of these are zero, we will have a divide by zero error
    // but notice that if they are zero, the vertices are collinear in projection and the triangle is edge on
    // we can render that as a line, but the better solution is to render nothing.  In a surface, the adjacent
    // triangles will eventually take care of it
    if ((distance0 == 0) || (distance1 == 0) || (distance2 == 0))
        return; 

    // create a fragment for reuse
    fragmentWithAttributes rasterFragment;

    for (rasterFragment.row = minY; rasterFragment.row <= maxY; rasterFragment.row++)
        { // per row
            // this is here so that clipping works correctly
            if (rasterFragment.row < 0) continue;
            if (rasterFragment.row >= frameBuffer.height) continue;
            for (rasterFragment.col = minX; rasterFragment.col <= maxX; rasterFragment.col++)
            { // per pixel
                // this is also for correct clipping
                if (rasterFragment.col < 0) continue;
                if (rasterFragment.col >= frameBuffer.width) continue;
            
                // the pixel in cartesian format
                Cartesian3 pixel(rasterFragment.col, rasterFragment.row, 0.0);
            
                // right - we have a pixel inside the frame buffer AND the bounding box
                // note we *COULD* compute gamma = 1.0 - alpha - beta instead
                float alpha = (normal12.dot(pixel) - lineConstant12) / distance0;           
                float beta = (normal20.dot(pixel) - lineConstant20) / distance1;            
                float gamma = (normal01.dot(pixel) - lineConstant01) / distance2;           

                // now perform the half-plane test
                if ((alpha < 0.0) || (beta < 0.0) || (gamma < 0.0))
                    continue;

                // Apply texture if enabled
                if(texture2DEnable)
                {
                    // Perform Bilinear interpolation on triangle
                    // Find the current point inside the triangle
                    // Using alpha, beta and gamma produced from barycentric interpolation
                    int midTextureX = alpha * vertex0.textureCoordX + beta * vertex1.textureCoordX + gamma * vertex2.textureCoordX;
                    int midTextureY = alpha * vertex0.textureCoordY + beta * vertex1.textureCoordY + gamma * vertex2.textureCoordY;

                    float sParm = (alpha * vertex0.textureCoordX + beta * vertex1.textureCoordX + gamma * vertex2.textureCoordX) - midTextureX;
                    float tParm = (alpha * vertex0.textureCoordY + beta * vertex1.textureCoordY + gamma * vertex2.textureCoordY) - midTextureY;
            
                    RGBAValue colour00 = textureBuffer[midTextureY][midTextureX];
                    RGBAValue colour01 = textureBuffer[midTextureY][midTextureX+1];
                    RGBAValue colour10 = textureBuffer[midTextureY+1][midTextureX];
                    RGBAValue colour11 = textureBuffer[midTextureY+1][midTextureX+1];

                    RGBAValue colour0 = colour00 + tParm * (colour01 - colour00);
                    RGBAValue colour1 = colour10 + tParm * (colour11 - colour10);

                    RGBAValue finalColour = colour1 + sParm * (colour1 - colour0);

                    // Mix colour if FAKEGL_MODULATE is enabled
                    if(fakeTextureMode == FAKEGL_MODULATE)
                    {
                        RGBAValue computedColour = alpha * vertex0.colour + beta * vertex1.colour + gamma * vertex2.colour;
                        finalColour = finalColour.modulate(computedColour);
                    }

                    // compute colour
                    rasterFragment.colour = finalColour;
                }
                else
                {
                    // compute colour
                    rasterFragment.colour = alpha * vertex0.colour + beta * vertex1.colour + gamma * vertex2.colour; 
                }

                // Phong shading
                if(phongShadingEnable)
                {  
                    for(int i = 0; i < 4; i++)
                    {
                        // Ambient
                        rasterFragment.ambientReflectivity[i]
                            = alpha * vertex0.ambientReflectivity[i] + 
                            beta * vertex1.ambientReflectivity[i] +
                            gamma * vertex2.ambientReflectivity[i];

                        // Diffuse
                        rasterFragment.diffuseReflectivity[i]
                            = alpha * vertex0.diffuseReflectivity[i] + 
                            beta * vertex1.diffuseReflectivity[i] +
                            gamma * vertex2.diffuseReflectivity[i];

                        // Specular
                        rasterFragment.specularReflectivity[i]
                            = alpha * vertex0.specularReflectivity[i] + 
                            beta * vertex1.specularReflectivity[i] +
                            gamma * vertex2.specularReflectivity[i];
                    }

                    float emissiveResult[4];

                    if(fakeTextureMode == FAKEGL_MODULATE)
                    {
                        emissiveResult[0] = emissiveColour[0] * rasterFragment.colour.red / 255.0f;
                        emissiveResult[1] = emissiveColour[1] * rasterFragment.colour.green / 255.0f;
                        emissiveResult[2] = emissiveColour[2] * rasterFragment.colour.blue / 255.0f;
                        emissiveResult[3] = emissiveColour[3] * rasterFragment.colour.alpha / 255.0f;
                    }
                    else
                    {
                        emissiveResult[0] = emissiveColour[0];
                        emissiveResult[1] = emissiveColour[1];
                        emissiveResult[2] = emissiveColour[2];
                        emissiveResult[3] = emissiveColour[3];
                    }

                    // Specular Coefficient
                    rasterFragment.specularCoefficient
                        = alpha * vertex0.specularCoefficient + 
                        beta * vertex1.specularCoefficient +
                        gamma * vertex2.specularCoefficient;

                    // Ambient Light calculations
                    float ambientResult[4];
                    if(fakeTextureMode == FAKEGL_MODULATE)
                    {
                        ambientResult[0] = (rasterFragment.colour.red / 255.0f) * ambientColour[0] * rasterFragment.ambientReflectivity[0];
                        ambientResult[1] = (rasterFragment.colour.green / 255.0f) * ambientColour[1] * rasterFragment.ambientReflectivity[1];
                        ambientResult[2] = (rasterFragment.colour.blue / 255.0f) * ambientColour[2] * rasterFragment.ambientReflectivity[2];
                        ambientResult[3] = (rasterFragment.colour.alpha / 255.0f) * ambientColour[3] * rasterFragment.ambientReflectivity[3];
                    }
                    else
                    {   
                        ambientResult[0] = ambientColour[0] * rasterFragment.ambientReflectivity[0];
                        ambientResult[1] = ambientColour[1] * rasterFragment.ambientReflectivity[1];
                        ambientResult[2] = ambientColour[2] * rasterFragment.ambientReflectivity[2];
                        ambientResult[3] = ambientColour[3] * rasterFragment.ambientReflectivity[3];
                    }
                
                    // Diffuse Light Calculations
                    float diffuseResult[4];

                    // Calculate cosine theta first
                    double cosTheta = 0.0;
                    Cartesian3 lightDirection = alpha * vertex0.fragLightVector
                        + beta * vertex1.fragLightVector + gamma * vertex2.fragLightVector;     

                    rasterFragment.normal = alpha * vertex0.normal + beta * vertex1.normal
                        + gamma * vertex2.normal;

                    Cartesian3 normalisedNormal = rasterFragment.normal.unit();

                    float normalNorm = normalisedNormal.length();
                    float lightDirectionNorm = lightDirection.length();

                    float diffuseDotProduct = normalisedNormal.dot(lightDirection);

                    // Check for negative dot product to prevent
                    // the object from absorbing light
                    if(diffuseDotProduct < 0)
                        diffuseDotProduct = 0;

                    cosTheta = diffuseDotProduct / (normalNorm * lightDirectionNorm);

                    if(fakeTextureMode == FAKEGL_MODULATE)
                    {
                        diffuseResult[0] = (rasterFragment.colour.red / 255.0f) * diffuseColour[0] * rasterFragment.diffuseReflectivity[0] * cosTheta;
                        diffuseResult[1] = (rasterFragment.colour.green / 255.0f) * diffuseColour[1] * rasterFragment.diffuseReflectivity[1] * cosTheta;
                        diffuseResult[2] = (rasterFragment.colour.blue / 255.0f) * diffuseColour[2] * rasterFragment.diffuseReflectivity[2] * cosTheta;
                        diffuseResult[3] = (rasterFragment.colour.alpha / 255.0f) * diffuseColour[3] * rasterFragment.diffuseReflectivity[3] * cosTheta;
                    }
                    else
                    {
                        diffuseResult[0] = diffuseColour[0] * rasterFragment.diffuseReflectivity[0] * cosTheta;
                        diffuseResult[1] = diffuseColour[1] * rasterFragment.diffuseReflectivity[1] * cosTheta;
                        diffuseResult[2] = diffuseColour[2] * rasterFragment.diffuseReflectivity[2] * cosTheta;
                        diffuseResult[3] = diffuseColour[3] * rasterFragment.diffuseReflectivity[3] * cosTheta;
                    }

                    // Specular Reflection calculations
                    float specularResult[4];

                    double cosThetaSpecular = 0.0;

                    // Eye coordinate
                    Cartesian3 eyeVector = alpha * vertex0.fragEyeVector
                        + beta * vertex1.fragEyeVector + gamma * vertex2.fragEyeVector;

                    Cartesian3 eyeDirection = eyeVector.unit();

                    Cartesian3 bisector = (lightDirection + eyeDirection)/2;
                    //Cartesian3 normalisedBisector = bisector.unit();

                    float bisectorNorm = bisector.length();

                    float specularDotProduct = normalisedNormal.dot(bisector);

                    if(specularDotProduct < 0)
                        specularDotProduct = 0;

                    cosThetaSpecular = specularDotProduct
                                        / (normalNorm * bisectorNorm);

                    if(fakeTextureMode == FAKEGL_MODULATE)
                    {
                        specularResult[0] = (rasterFragment.colour.red / 255.0f) * specularColour[0] 
                                        * rasterFragment.specularReflectivity[0] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                        
                        specularResult[1] = (rasterFragment.colour.green / 255.0f) * specularColour[1] 
                                        * rasterFragment.specularReflectivity[1] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                        
                        specularResult[2] = (rasterFragment.colour.blue / 255.0f) * specularColour[2] 
                                        * rasterFragment.specularReflectivity[2] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                        
                        specularResult[3] = (rasterFragment.colour.alpha / 255.0f) * specularColour[3] 
                                        * rasterFragment.specularReflectivity[3] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                    }
                    else
                    {
                        specularResult[0] = specularColour[0] 
                                        * rasterFragment.specularReflectivity[0] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                    
                        specularResult[1] = specularColour[1] 
                                        * rasterFragment.specularReflectivity[1] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                        
                        specularResult[2] = specularColour[2] 
                                        * rasterFragment.specularReflectivity[2] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                        
                        specularResult[3] = specularColour[3] 
                                        * rasterFragment.specularReflectivity[3] 
                                        * std::pow(cosThetaSpecular, rasterFragment.specularCoefficient);
                    }
                
                    // Adding all values from emissive, ambient, diffuse and specular together
                    float allResult[4];
                    allResult[0] = (emissiveResult[0] + ambientResult[0] 
                        + diffuseResult[0] + specularResult[0]);
                    allResult[1] = (emissiveResult[1] + ambientResult[1] 
                        + diffuseResult[1] + specularResult[1]);
                    allResult[2] = (emissiveResult[2] + ambientResult[2] 
                        + diffuseResult[2] + specularResult[2]);
                    allResult[3] = (emissiveResult[3] + ambientResult[3] 
                        + diffuseResult[3] + specularResult[3]);

                    // Clamp the values for RGBA channel
                    if(allResult[0] < 0)
                        rasterFragment.colour.red = 0;
                    else if(allResult[0] > 1)
                        rasterFragment.colour.red = 255;
                    else
                        rasterFragment.colour.red = allResult[0] * 255;

                    if(allResult[1] < 0)
                        rasterFragment.colour.green = 0;
                    else if(allResult[1] > 1)
                        rasterFragment.colour.green = 255;
                    else
                        rasterFragment.colour.green = allResult[1] * 255;

                    if(allResult[2] < 0)
                        rasterFragment.colour.blue = 0;
                    else if(allResult[2] > 1)
                        rasterFragment.colour.blue = 255;
                    else
                        rasterFragment.colour.blue = allResult[2] * 255;

                    if(allResult[3] < 0)
                        rasterFragment.colour.alpha = 0;
                    else if(allResult[3] > 1)
                        rasterFragment.colour.alpha = 255;
                    else
                        rasterFragment.colour.alpha = allResult[3] * 255;
                }

                // Perform depth test if enabled
                if(depthTestEnable)
                {
                    float depth = alpha * vertex0.position.z 
                                + beta * vertex1.position.z 
                                + gamma * vertex2.position.z;
                    if(depth < depthBuffer[rasterFragment.row][rasterFragment.col].alpha)
                    {
                        depthBuffer[rasterFragment.row][rasterFragment.col].alpha = depth;
                        // now we add it to the queue for fragment processing
                        fragmentQueue.push_back(rasterFragment);
                    }
                }
                else
                {
                    // now we add it to the queue for fragment processing
                    fragmentQueue.push_back(rasterFragment);
                }
            } // per pixel
        } // per row
} // RasteriseTriangle()

// process a single fragment
void FakeGL::ProcessFragment()
    { // ProcessFragment()
        for(fragmentWithAttributes f:fragmentQueue)
        {
            frameBuffer[f.row][f.col].red = f.colour.red;
            frameBuffer[f.row][f.col].green = f.colour.green;
            frameBuffer[f.row][f.col].blue = f.colour.blue;
            frameBuffer[f.row][f.col].alpha = f.colour.alpha;
        }
        // Clear the fragment queue after use
        fragmentQueue.clear();
    } // ProcessFragment()

// standard routine for dumping the entire FakeGL context (except for texture / image)
std::ostream &operator << (std::ostream &outStream, FakeGL &fakeGL)
    { // operator <<
    outStream << "=========================" << std::endl;
    outStream << "Dumping FakeGL Context   " << std::endl;
    outStream << "=========================" << std::endl;


    outStream << "-------------------------" << std::endl;
    outStream << "Vertex Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.vertexQueue.begin(); vertex < fakeGL.vertexQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.vertexQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Raster Queue:            " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto vertex = fakeGL.rasterQueue.begin(); vertex < fakeGL.rasterQueue.end(); vertex++)
        { // per matrix
        outStream << "Vertex " << vertex - fakeGL.rasterQueue.begin() << std::endl;
        outStream << *vertex;
        } // per matrix


    outStream << "-------------------------" << std::endl;
    outStream << "Fragment Queue:          " << std::endl;
    outStream << "-------------------------" << std::endl;
    for (auto fragment = fakeGL.fragmentQueue.begin(); fragment < fakeGL.fragmentQueue.end(); fragment++)
        { // per matrix
        outStream << "Fragment " << fragment - fakeGL.fragmentQueue.begin() << std::endl;
        outStream << *fragment;
        } // per matrix


    return outStream;
    } // operator <<

// subroutines for other classes
std::ostream &operator << (std::ostream &outStream, vertexWithAttributes &vertex)
    { // operator <<
    std::cout << "Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

	// you

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, screenVertexWithAttributes &vertex) 
    { // operator <<
    std::cout << "Screen Vertex With Attributes" << std::endl;
    std::cout << "Position:   " << vertex.position << std::endl;
    std::cout << "Colour:     " << vertex.colour << std::endl;

    return outStream;
    } // operator <<

std::ostream &operator << (std::ostream &outStream, fragmentWithAttributes &fragment)
    { // operator <<
    std::cout << "Fragment With Attributes" << std::endl;
    std::cout << "Row:        " << fragment.row << std::endl;
    std::cout << "Col:        " << fragment.col << std::endl;
    std::cout << "Colour:     " << fragment.colour << std::endl;

    return outStream;
    } // operator <<


    
    