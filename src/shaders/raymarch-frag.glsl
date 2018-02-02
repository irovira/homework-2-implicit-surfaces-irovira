#version 300 es

precision highp float;
uniform vec2 u_Resolution;
uniform float u_Time;

in vec2 fs_Pos;

out vec4 out_Col;

const int MAX_MARCHING_STEPS = 255;
const float MIN_DIST = 0.0;
const float MAX_DIST = 100.0;
const float EPSILON = 0.0001;

/**
 * Rotation matrix around the X axis.
 */
mat3 rotateX(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(1, 0, 0),
        vec3(0, c, -s),
        vec3(0, s, c)
    );
}

/**
 * Rotation matrix around the Y axis.
 */
mat3 rotateY(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, 0, s),
        vec3(0, 1, 0),
        vec3(-s, 0, c)
    );
}

/**
 * Rotation matrix around the Z axis.
 */
mat3 rotateZ(float theta) {
    float c = cos(theta);
    float s = sin(theta);
    return mat3(
        vec3(c, -s, 0),
        vec3(s, c, 0),
        vec3(0, 0, 1)
    );
}

/**
 * Constructive solid geometry intersection operation on SDF-calculated distances.
 */
float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

/**
 * Constructive solid geometry union operation on SDF-calculated distances.
 */
float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

/**
 * Constructive solid geometry difference operation on SDF-calculated distances.
 */
float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}

/**
 * Signed distance function for a cube centered at the origin
 * with dimensions specified by size.
 */
float boxSDF(vec3 p, vec3 size) {
    vec3 d = abs(p) - (size / 2.0);
    
    // Assuming p is inside the cube, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(d.x, max(d.y, d.z)), 0.0);
    
    // Assuming p is outside the cube, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(d, 0.0));
    
    return insideDistance + outsideDistance;
}

/**
 * Signed distance function for a sphere centered at the origin with radius r.
 */
float sphereSDF(vec3 p, float r) {
    return length(p) - r;
}

/**
 * Signed distance function for an XY aligned cylinder centered at the origin with
 * height h and radius r.
 */
float cylinderSDF(vec3 p, float h, float r) {
    // How far inside or outside the cylinder the point is, radially
    float inOutRadius = length(p.xy) - r;
    
    // How far inside or outside the cylinder is, axially aligned with the cylinder
    float inOutHeight = abs(p.z) - h/2.0;
    
    // Assuming p is inside the cylinder, how far is it from the surface?
    // Result will be negative or zero.
    float insideDistance = min(max(inOutRadius, inOutHeight), 0.0);

    // Assuming p is outside the cylinder, how far is it from the surface?
    // Result will be positive or zero.
    float outsideDistance = length(max(vec2(inOutRadius, inOutHeight), 0.0));
    
    return insideDistance + outsideDistance;
}

/**
 * Signed distance function describing the scene.
 * 
 * Absolute value of the return value indicates the distance to the surface.
 * Sign indicates whether the point is inside or outside the surface,
 * negative indicating inside.
 */
 float innerLotus(vec3 samplePoint) {    
    // Slowly spin the whole scene
    
    samplePoint = samplePoint + vec3(0.0f, -0.25f, 0.0f); 
    samplePoint = rotateY(u_Time / 2.0) * samplePoint;
    float radius = 1.0f;
    float cut = radius * 1.1f;
	vec3 dir = vec3(0.0f,0.0f, radius);
	vec3 dir_Cut = vec3(0.0f,-0.15f, radius);
    float sphere = sphereSDF(samplePoint + dir, radius);
	float sphere_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere_Diff = differenceSDF(sphere, sphere_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere2 = sphereSDF(samplePoint +  dir, radius);
	float sphere2_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere2_Diff = differenceSDF(sphere2, sphere2_Cut);

	float petal1 = intersectSDF(sphere_Diff, sphere2_Diff);

// petal 2
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere3 = sphereSDF(samplePoint + dir, radius);
	float sphere3_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere3_Diff = differenceSDF(sphere3, sphere3_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere4 = sphereSDF(samplePoint +  dir, radius);
	float sphere4_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere4_Diff = differenceSDF(sphere4, sphere4_Cut);

	float petal2 = intersectSDF(sphere3_Diff, sphere4_Diff);

	float group1 = unionSDF(petal1,petal2);
//GROUP 2
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 3
    float sphere5 = sphereSDF(samplePoint + dir, radius);
	float sphere5_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere5_Diff = differenceSDF(sphere5, sphere5_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere6 = sphereSDF(samplePoint +  dir, radius);
	float sphere6_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere6_Diff = differenceSDF(sphere6, sphere6_Cut);

	float petal3 = intersectSDF(sphere5_Diff, sphere6_Diff);

// petal 4
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere7 = sphereSDF(samplePoint + dir, radius);
	float sphere7_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere7_Diff = differenceSDF(sphere7, sphere7_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere8 = sphereSDF(samplePoint +  dir, radius);
	float sphere8_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere8_Diff = differenceSDF(sphere8, sphere8_Cut);

	float petal4 = intersectSDF(sphere7_Diff, sphere8_Diff);

	float group2 = unionSDF(petal3,petal4);

	float side1 = unionSDF(group1,group2);
//GROUP 3
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 5
    float sphere9 = sphereSDF(samplePoint + dir, radius);
	float sphere9_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere9_Diff = differenceSDF(sphere9, sphere9_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere10 = sphereSDF(samplePoint +  dir, radius);
	float sphere10_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere10_Diff = differenceSDF(sphere10, sphere10_Cut);

	float petal5 = intersectSDF(sphere9_Diff, sphere10_Diff);

// petal 6
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere11 = sphereSDF(samplePoint + dir, radius);
	float sphere11_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere11_Diff = differenceSDF(sphere11, sphere11_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere12 = sphereSDF(samplePoint +  dir, radius);
	float sphere12_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere12_Diff = differenceSDF(sphere12, sphere12_Cut);

	float petal6 = intersectSDF(sphere11_Diff, sphere12_Diff);

	float group3 = unionSDF(petal5, petal6);
//GROUP 4
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 6
    float sphere13 = sphereSDF(samplePoint + dir, radius);
	float sphere13_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere13_Diff = differenceSDF(sphere13, sphere13_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere14 = sphereSDF(samplePoint +  dir, radius);
	float sphere14_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere14_Diff = differenceSDF(sphere14, sphere14_Cut);

	float petal7 = intersectSDF(sphere13_Diff, sphere14_Diff);

// petal 7
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere15 = sphereSDF(samplePoint + dir, radius);
	float sphere15_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere15_Diff = differenceSDF(sphere15, sphere15_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere16 = sphereSDF(samplePoint +  dir, radius);
	float sphere16_Cut = sphereSDF(samplePoint + dir_Cut,cut);
	float sphere16_Diff = differenceSDF(sphere16, sphere16_Cut);

	float petal8 = intersectSDF(sphere15_Diff, sphere16_Diff);

	float group4= unionSDF(petal7, petal8);

	float side2 = unionSDF(group3, group4);

    
    // float ballOffset = 0.4 + 1.0 + sin(1.7 * u_Time);
    // float ballRadius = 0.3;
    // float balls = sphereSDF(samplePoint - vec3(ballOffset, 0.0, 0.0), ballRadius);
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(ballOffset, 0.0, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, ballOffset, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(0.0, ballOffset, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, 0.0, ballOffset), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(0.0, 0.0, ballOffset), ballRadius));
    
    return  unionSDF(side1,side2);//unionSDF(group1,group2);
    
    // float csgNut = differenceSDF(intersectSDF(cube, sphere),
    //                      unionSDF(cylinder1, unionSDF(cylinder2, cylinder3)));
    
    //return unionSDF(differenceSDF(cylinder2, sphere), differenceSDF(cylinder21, sphere21)); //unionSDF(cylinder1, unionSDF(cylinder2, cylinder3));
}
float mainLotus(vec3 samplePoint) {    
    // Slowly spin the whole scene
    samplePoint = rotateY(-u_Time / 2.0) * samplePoint;
    
    float cylinderRadius = 1.0f; //0.4 + (1.0 - 0.4) * (1.0 + sin(1.7 * u_Time)) / 2.0;
    float cylinder1 = cylinderSDF(samplePoint, 2.0, cylinderRadius);
    float cylinder2 = cylinderSDF(rotateX(radians(90.0)) * samplePoint, 2.0, cylinderRadius);
	float cylinder21 = cylinderSDF(rotateX(radians(90.0)) * (samplePoint + vec3(0.0f,-0.25f,0.0f)), 2.0, cylinderRadius * 0.5);
    float cylinder3 = cylinderSDF(rotateY(radians(90.0)) * samplePoint, 2.0, cylinderRadius);
    
    float cube = boxSDF(samplePoint, vec3(1.8, 1.8, 1.8));
    
	vec3 dir = vec3(0.0f,0.0f, 1.5f);
	vec3 dir_Cut = vec3(0.0f,-0.25f, 1.5f);
    float sphere = sphereSDF(samplePoint + dir, 1.5);
	float sphere_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere_Diff = differenceSDF(sphere, sphere_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere2 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere2_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere2_Diff = differenceSDF(sphere2, sphere2_Cut);

	float petal1 = intersectSDF(sphere_Diff, sphere2_Diff);

	float sphere21 = sphereSDF(samplePoint + vec3(0.0f, 0.0f,-1.5f), 1.5);
// petal 2
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere3 = sphereSDF(samplePoint + dir, 1.5);
	float sphere3_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere3_Diff = differenceSDF(sphere3, sphere3_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere4 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere4_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere4_Diff = differenceSDF(sphere4, sphere4_Cut);

	float petal2 = intersectSDF(sphere3_Diff, sphere4_Diff);

	float group1 = unionSDF(petal1,petal2);
//GROUP 2
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 3
    float sphere5 = sphereSDF(samplePoint + dir, 1.5);
	float sphere5_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere5_Diff = differenceSDF(sphere5, sphere5_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere6 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere6_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere6_Diff = differenceSDF(sphere6, sphere6_Cut);

	float petal3 = intersectSDF(sphere5_Diff, sphere6_Diff);

// petal 4
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere7 = sphereSDF(samplePoint + dir, 1.5);
	float sphere7_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere7_Diff = differenceSDF(sphere7, sphere7_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere8 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere8_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere8_Diff = differenceSDF(sphere8, sphere8_Cut);

	float petal4 = intersectSDF(sphere7_Diff, sphere8_Diff);

	float group2 = unionSDF(petal3,petal4);

	float side1 = unionSDF(group1,group2);
//GROUP 3
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 5
    float sphere9 = sphereSDF(samplePoint + dir, 1.5);
	float sphere9_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere9_Diff = differenceSDF(sphere9, sphere9_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere10 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere10_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere10_Diff = differenceSDF(sphere10, sphere10_Cut);

	float petal5 = intersectSDF(sphere9_Diff, sphere10_Diff);

// petal 6
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere11 = sphereSDF(samplePoint + dir, 1.5);
	float sphere11_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere11_Diff = differenceSDF(sphere11, sphere11_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere12 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere12_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere12_Diff = differenceSDF(sphere12, sphere12_Cut);

	float petal6 = intersectSDF(sphere11_Diff, sphere12_Diff);

	float group3 = unionSDF(petal5, petal6);
//GROUP 4
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;
//petal 6
    float sphere13 = sphereSDF(samplePoint + dir, 1.5);
	float sphere13_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere13_Diff = differenceSDF(sphere13, sphere13_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere14 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere14_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere14_Diff = differenceSDF(sphere14, sphere14_Cut);

	float petal7 = intersectSDF(sphere13_Diff, sphere14_Diff);

// petal 7
	dir = rotateY(radians(40.0)) * dir;
	dir_Cut = rotateY(radians(40.0)) * dir_Cut;

    float sphere15 = sphereSDF(samplePoint + dir, 1.5);
	float sphere15_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere15_Diff = differenceSDF(sphere15, sphere15_Cut);

	dir = rotateY(radians(5.0)) * dir;
	dir_Cut = rotateY(radians(5.0)) * dir_Cut;

	float sphere16 = sphereSDF(samplePoint +  dir, 1.5);
	float sphere16_Cut = sphereSDF(samplePoint + dir_Cut,1.65);
	float sphere16_Diff = differenceSDF(sphere16, sphere16_Cut);

	float petal8 = intersectSDF(sphere15_Diff, sphere16_Diff);

	float group4= unionSDF(petal7, petal8);

	float side2 = unionSDF(group3, group4);

    
    // float ballOffset = 0.4 + 1.0 + sin(1.7 * u_Time);
    // float ballRadius = 0.3;
    // float balls = sphereSDF(samplePoint - vec3(ballOffset, 0.0, 0.0), ballRadius);
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(ballOffset, 0.0, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, ballOffset, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(0.0, ballOffset, 0.0), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, 0.0, ballOffset), ballRadius));
    // balls = unionSDF(balls, sphereSDF(samplePoint + vec3(0.0, 0.0, ballOffset), ballRadius));
    
    return  unionSDF(side1,side2);//unionSDF(group1,group2);
    
}

float ball(vec3 samplePoint){
    float cylinderRadius = 0.25f; //0.4 + (1.0 - 0.4) * (1.0 + sin(1.7 * u_Time)) / 2.0;
    float cylinder1 = cylinderSDF(rotateX(radians(90.0)) * samplePoint, 1.3, cylinderRadius * 1.3);
    float cylinder2 = cylinderSDF(rotateX(radians(90.0)) * samplePoint, 1.3, cylinderRadius / 2.0);
    float cylinder = differenceSDF(cylinder1, cylinder2);
    float ballOffset = 0.3 + (sin(22.0 * u_Time) / 2.1f);
    float maxOffset = 0.3 + (1.0f / 2.1f);
    float ballRadius = 0.3;
    float balls = sphereSDF(samplePoint -  vec3(maxOffset, 0.0, 0.0) + vec3(ballOffset, 0.0, 0.0), ballRadius * 0.5);
    
    balls = unionSDF(balls, sphereSDF(samplePoint + vec3(maxOffset, 0.0, 0.0) - vec3(ballOffset, 0.0, 0.0), ballRadius * 0.5));
    //balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, maxOffset, 0.0) + vec3(0.0, ballOffset, 0.0), ballRadius * 0.5));
    balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, ballOffset, 0.0), ballRadius * 0.8));
    balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, 0.0, maxOffset) + vec3(0.0, 0.0, ballOffset), ballRadius * 0.5));
    balls = unionSDF(balls, sphereSDF(samplePoint + vec3(0.0, 0.0, maxOffset) - vec3(0.0, 0.0, ballOffset), ballRadius * 0.5));
    return balls;//differenceSDF(balls, cylinder);
    //balls = unionSDF(balls, sphereSDF(samplePoint - vec3(0.0, ballOffset, 0.0), ballRadius));
}

float opRep( vec3 p, vec3 c )
{
    vec3 q = mod(p,c)-0.5*c;
    return boxSDF(q,vec3(1.0f));
}

float sceneSDF(vec3 samplePoint){
    float flower1 = unionSDF(mainLotus(samplePoint), innerLotus(samplePoint));
    float stuff = unionSDF(flower1, ball(samplePoint));
    vec3 boxStart = vec3(0.0f,0.0f,0.0f);
    vec3 boxSize = vec3(2.0f, 2.0f, 0.0f);
    //samplePoint = samplePoint + vec3(0.0f,-1.0f, 5.0f);
    //float flower2 = unionSDF(mainLotus(samplePoint), innerLotus(samplePoint));
    return unionSDF(stuff, opRep(samplePoint + vec3(0.0,0.0,5.0), boxSize));
}

/**
 * Return the shortest distance from the eyepoint to the scene surface along
 * the marching direction. If no part of the surface is found between start and end,
 * return end.
 * 
 * eye: the eye point, acting as the origin of the ray
 * marchingDirection: the normalized direction to march in
 * start: the starting distance away from the eye
 * end: the max distance away from the ey to march before giving up
 */
float shortestDistanceToSurface(vec3 eye, vec3 marchingDirection, float start, float end) {
    float depth = start;
    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        float dist = sceneSDF(eye + depth * marchingDirection);
        if (dist < EPSILON) {
			return depth;
        }
        depth += dist;
        if (depth >= end) {
            return end;
        }
    }
    return end;
}
            

/**
 * Return the normalized direction to march in from the eye point for a single pixel.
 * 
 * fieldOfView: vertical field of view in degrees
 * size: resolution of the output image
 * fragCoord: the x,y coordinate of the pixel in the output image
 */
vec3 rayDirection(float fieldOfView, vec2 size, vec2 fragCoord) {
    vec2 xy = fragCoord - size / 2.0;
    float z = size.y / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

/**
 * Using the gradient of the SDF, estimate the normal on the surface at point p.
 */
vec3 estimateNormal(vec3 p) {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

/**
 * Lighting contribution of a single point light source via Phong illumination.
 * 
 * The vec3 returned is the RGB color of the light's contribution.
 *
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 * lightPos: the position of the light
 * lightIntensity: color/intensity of the light
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongContribForLight(vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye,
                          vec3 lightPos, vec3 lightIntensity) {
    vec3 N = estimateNormal(p);
    vec3 L = normalize(lightPos - p);
    vec3 V = normalize(eye - p);
    vec3 R = normalize(reflect(-L, N));
    
    float dotLN = dot(L, N);
    float dotRV = dot(R, V);
    
    if (dotLN < 0.0) {
        // Light not visible from this point on the surface
        return vec3(0.0, 0.0, 0.0);
    } 
    
    if (dotRV < 0.0) {
        // Light reflection in opposite direction as viewer, apply only diffuse
        // component
        return lightIntensity * (k_d * dotLN);
    }
    return lightIntensity * (k_d * dotLN + k_s * pow(dotRV, alpha));
}

/**
 * Lighting via Phong illumination.
 * 
 * The vec3 returned is the RGB color of that point after lighting is applied.
 * k_a: Ambient color
 * k_d: Diffuse color
 * k_s: Specular color
 * alpha: Shininess coefficient
 * p: position of point being lit
 * eye: the position of the camera
 *
 * See https://en.wikipedia.org/wiki/Phong_reflection_model#Description
 */
vec3 phongIllumination(vec3 k_a, vec3 k_d, vec3 k_s, float alpha, vec3 p, vec3 eye) {
    const vec3 ambientLight = 0.5 * vec3(1.0, 1.0, 1.0);
    vec3 color = ambientLight * k_a;
    
    vec3 light1Pos = vec3(4.0 * sin(u_Time),
                          2.0,
                          4.0 * cos(u_Time));
    vec3 light1Intensity = vec3(0.4, 0.4, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light1Pos,
                                  light1Intensity);
    
    vec3 light2Pos = vec3(2.0 * sin(0.37 * u_Time),
                          2.0 * cos(0.37 * u_Time),
                          2.0);
    vec3 light2Intensity = vec3(0.4, 0.4, 0.4);
    
    color += phongContribForLight(k_d, k_s, alpha, p, eye,
                                  light2Pos,
                                  light2Intensity);    
    return color;
}

/**
 * Return a transform matrix that will transform a ray from view space
 * to world coordinates, given the eye point, the camera target, and an up vector.
 *
 * This assumes that the center of the camera is aligned with the negative z axis in
 * view space when calculating the ray marching direction. See rayDirection.
 */
mat3 viewMatrix(vec3 eye, vec3 center, vec3 up) {
    // Based on gluLookAt man page
    vec3 f = normalize(center - eye);
    vec3 s = normalize(cross(f, up));
    vec3 u = cross(s, f);
    return mat3(s, u, -f);
}

void main()
{
	vec3 viewDir = rotateX(radians(5.0f)) * rayDirection(45.0, u_Resolution, vec2(((fs_Pos.x + 1.0f ) / 2.0f) * u_Resolution.x, ((fs_Pos.y + 1.0f ) / 2.0f) * u_Resolution.y));
    //vec3 eye = vec3(8.0, 5.0 * sin(0.2 * u_Time), 7.0);
    vec3 eye = vec3(0.0, 5.0 * sin(u_Time), 12.0);
    
    mat3 viewToWorld = viewMatrix(eye, vec3(0.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0));
    
    vec3 worldDir = viewToWorld * viewDir;
    
    float dist = shortestDistanceToSurface(eye, worldDir, MIN_DIST, MAX_DIST);
    
    if (dist > MAX_DIST - EPSILON) {
        // Didn't hit anything
        out_Col = vec4(0.0, 0.0, 0.0, 1.0);
		return;
    }
    
    // The closest point on the surface to the eyepoint along the view ray
    vec3 p = eye + dist * worldDir;
    
    // Use the surface normal as the ambient color of the material
    vec3 K_a = (estimateNormal(p) + vec3(1.0)) / 2.0;
    vec3 K_d = K_a;
    vec3 K_s = vec3(1.0, 1.0, 1.0);
    float shininess = 10.0;
    
    vec3 color = phongIllumination(K_a, K_d, K_s, shininess, p, eye);
    
    out_Col = vec4(color, 1.0);
}