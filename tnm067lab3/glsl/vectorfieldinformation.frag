/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2016 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/
 
#include "utils/structs.glsl"

uniform sampler2D vfColor;
uniform ImageParameters vfParameters;
in vec3 texCoord_;


float passThrough(vec2 coord){
    return texture(vfColor,coord).x;
}

float magnitude( vec2 coord ){
	//float mag = abs(sqrt(pow(coord.x,2)+pow(coord.y,2))); //For 3D do the same for the z-component

    //TODO find the magnitude of the vectorfield at the position coords
	vec2 velo = texture2D(vfColor,coord).xy;

    return length(velo);
}

vec2 derivative_x(vec2 coord, vec2 pixelSize){
	vec2 velo1 = texture(vfColor,coord + vec2(pixelSize.x,0)).xy;
	vec2 velo2 = texture(vfColor,coord - vec2(pixelSize.x,0)).xy;

	vec2 d = (velo1 - velo2)/(2.0*pixelSize.x);
	return d;

}

vec2 derivative_y(vec2 coord, vec2 pixelSize){
	vec2 velo1 = texture(vfColor,coord + vec2(0, pixelSize.y)).xy;
	vec2 velo2 = texture(vfColor,coord - vec2(0, pixelSize.y)).xy;

	vec2 d = (velo1 - velo2)/(2.0*pixelSize.y);
	return d;

}

float divergence(vec2 coord){
    //TODO find the divergence of the vectorfield at the position coords
	vec2 pixelSize = vfParameters.reciprocalDimensions;

	vec2 dx = derivative_x(coord,pixelSize);
	vec2 dy = derivative_y(coord,pixelSize);

	float div = dx.x + dy.y;

    return div;
}



float rotation(vec2 coord){
    //TODO find the curl of the vectorfield at the position coords
    vec2 pixelSize = vfParameters.reciprocalDimensions;
	vec2 velo = texture(vfColor,coord).xy;

	vec2 dx = derivative_x(coord,pixelSize);
	vec2 dy = derivative_y(coord,pixelSize);

	float rot = dx.y - dy.x;

    return rot;
}


void main(void) {
    float v = OUTPUT(texCoord_.xy);
    FragData0 = vec4(v);
}
