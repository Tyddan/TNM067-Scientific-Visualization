/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2017 Inviwo Foundation
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

#ifndef IVW_INTERPOLATIONMETHODS_H
#define IVW_INTERPOLATIONMETHODS_H

#include <modules/tnm067lab1/tnm067lab1moduledefine.h>
#include <inviwo/core/common/inviwo.h>

namespace inviwo {
    
    template<typename T> struct float_type{using type = double;};

    template<> struct float_type<float>{using type = float;};
    template<> struct float_type<vec3>{using type = float;};
    template<> struct float_type<vec2>{using type = float;};
    template<> struct float_type<vec4>{using type = float;};


    namespace TNM067{
        namespace Interpolation{
        


#define ENABLE_LINEAR_UNITTEST 1
    template<typename T, typename F = double> 
    T linear(const T &a, const T &b , F x){
        //if(x<=0) return a;
        //if(x>=1) return b;

        return a*(1 - x) + (b * x);
    }


    /*
     2------3
     |      |
    y|  �   |
     |      |
     0------1
        x
    */
#define ENABLE_BILINEAR_UNITTEST 1
    template<typename T, typename F = double> 
    T bilinear(const std::array<T, 4> &v, F x, F y) {
		T f1 = linear(v[0], v[1], x);
		T f2 = linear(v[2], v[3], x);
		T f3 = linear(f1, f2, y);


        return f3;
    }



    /* 
    a--�----b------c
    0  x    1      2
    */
#define ENABLE_QUADRATIC_UNITTEST 1
    template<typename T, typename F = double> 
    T quadratic(const T &a, const T &b , const T &c , F x){
		//return a + (b * x) + (c * pow(x, 2));
		T f = ((1 - x)*(1 - (2 * x))*a) + ((4 * x)*(1 - x)*b) + (x*((2 * x) - 1)*c);
		return f;
    }



    /* 
    6-------7-------8
    |       |       |
    |       |       |
    |       |       |
    3-------4-------5
    |       |       |
   y|  �    |       |
    |       |       |
    0-------1-------2
    0  x    1       2
    */
#define ENABLE_BIQUADRATIC_UNITTEST 1
    template<typename T, typename F = double> 
    T biQuadratic(const std::array<T,9> &v ,F x,F y){
		T f1 = quadratic(v[0], v[1], v[2], x);
		T f2 = quadratic(v[3], v[4], v[5], x);
		T f3 = quadratic(v[6], v[7], v[8], x);

		T f4 = quadratic(f1, f2, f3, y);


        return f4;
    }


   
    /*
     2---------3
     |'-.      |
     |   -,    |
   y |  �  -,  |
     |       -,|
     0---------1
        x
    */
#define ENABLE_BARYCENTRIC_UNITTEST 1
    template<typename T, typename F = double> 
    T barycentric(const std::array<T,4> &v ,F x,F y){
		float alpha, beta, gamma;
		

		
		
		if (x + y < 1.f) //012 triangle
		{
			alpha = 1.0f - (x + y);
			beta = x;
			gamma = y;
			

			T t2 = v[0] * alpha + v[1] * beta + v[2] * gamma;

			return t2;
		}
		else
		{
			alpha = (x + y) - 1.0f;
			beta = 1 - y;
			gamma = 1 - x;

			T t1 = v[3] * alpha + v[1] * beta + v[2] * gamma;

			return t1;
		 }
	}
			

} // namespace interpolation
} // namespace TNM067
} // namespace inviwo

#endif // IVW_INTERPOLATIONMETHODS_H

