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

#include <modules/tnm067lab2/processors/marchingtetrahedra.h>
#include <inviwo/core/datastructures/geometry/basicmesh.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/network/networklock.h>

namespace inviwo {

size_t MarchingTetrahedra::HashFunc::max = 1;

const ProcessorInfo MarchingTetrahedra::processorInfo_{
    "org.inviwo.MarchingTetrahedra",  // Class identifier
    "Marching Tetrahedra",            // Display name
    "TNM067",                         // Category
    CodeState::Experimental,          // Code state
    Tags::None,                       // Tags
};
const ProcessorInfo MarchingTetrahedra::getProcessorInfo() const { return processorInfo_; }

MarchingTetrahedra::MarchingTetrahedra()
    : Processor()
    , volume_("volume")
    , mesh_("mesh")
    , isoValue_("isoValue", "ISO value", 0.5f, 0.0f, 1.0f) {

    addPort(volume_);
    addPort(mesh_);

    addProperty(isoValue_);

    isoValue_.setSerializationMode(PropertySerializationMode::All);

    volume_.onChange([&]() {
        if (!volume_.hasData()) {
            return;
        }
        NetworkLock lock(getNetwork());
        const float iso = (isoValue_.get() - isoValue_.getMinValue()) /
                    (isoValue_.getMaxValue() - isoValue_.getMinValue());
        const vec2 vr = volume_.getData()->dataMap_.valueRange;
        isoValue_.setMinValue(vr.x);
        isoValue_.setMaxValue(vr.y);
        isoValue_.setIncrement(glm::abs(vr.y - vr.x) / 50.0f);
        isoValue_.set(iso * (vr.y - vr.x) + vr.x);
        isoValue_.setCurrentStateAsDefault();
    });
}

void MarchingTetrahedra::process() {
    auto volume = volume_.getData()->getRepresentation<VolumeRAM>();
    MeshHelper mesh(volume_.getData());

    const auto dims = volume->getDimensions();
    MarchingTetrahedra::HashFunc::max = dims.x * dims.y * dims.z;

    float iso = isoValue_.get();

    util::IndexMapper3D indexMapper(dims);

    const static size_t tetrahedraIds[6][4] = {{0, 1, 2, 5}, {1, 3, 2, 5}, {3, 2, 5, 7},
                                               {0, 2, 4, 5}, {6, 4, 2, 5}, {6, 7, 5, 2}};

    size3_t pos;
    for (pos.z = 0; pos.z < dims.z - 1; ++pos.z) {
        for (pos.y = 0; pos.y < dims.y - 1; ++pos.y) {
            for (pos.x = 0; pos.x < dims.x - 1; ++pos.x) {
                // Step 1: create current cell
                // Use volume->getAsDouble to query values from the volume
				
                // Spatial position should be between 0 and 1
                // The voxel index should be the 1D-index for the voxel
				int CELL_SIZE = 2;

                Cell c;


				for (int z = 0; z < CELL_SIZE; z++)
				{
					for (int y = 0; y < CELL_SIZE; y++)
					{
						for (int x = 0; x < CELL_SIZE; x++)
						{
							int index = (CELL_SIZE * CELL_SIZE*z) + (CELL_SIZE * y) + x;
							


							//position
							c.voxels[index].pos.x = (pos.x + x) / (dims.x-1.0);
							c.voxels[index].pos.y = (pos.y + y) / (dims.y-1.0);
							c.voxels[index].pos.z = (pos.z + z) / (dims.z-1.0); 
							
							size3_t query_pos{ pos.x + x, pos.y + y, pos.z + z };
							
							c.voxels[index].value = volume->getAsDouble(query_pos);
							c.voxels[index].index = indexMapper(query_pos);

							


						}
					}

				}



                // Step 2: Subdivide cell into tetrahedra (hint: use tetrahedraIds)
                std::vector<Tetrahedra> tetrahedras;
				Tetrahedra temp;

				for (size_t i = 0; i < 6; i++)
				{
					for (size_t j = 0; j < 4; j++)
					{
						temp.voxels[j] = c.voxels[tetrahedraIds[i][j]];
					}
					tetrahedras.push_back(temp);
				}

                for (const Tetrahedra& tetrahedra : tetrahedras) {
                    // Step three: Calculate for tetra case index
                    int caseId = 0;

					if (tetrahedra.voxels[0].value < iso) caseId += 1;  
					if (tetrahedra.voxels[1].value < iso) caseId += 2;
					if (tetrahedra.voxels[2].value < iso) caseId += 4;
					if (tetrahedra.voxels[3].value < iso) caseId += 8;

                    // step four: Extract triangles

					Voxel v0 = tetrahedra.voxels[0];
					Voxel v1 = tetrahedra.voxels[1];
					Voxel v2 = tetrahedra.voxels[2];
					Voxel v3 = tetrahedra.voxels[3];

					vec3 interVal0, interVal1, interVal2, interVal3;
					size_t id0, id1, id2, id3;

					switch (caseId) {
					case 0:
					case 15:
						break;
					case 1:
					case 14:
						/*
						interVal0 = v0.pos + (v1.pos - v0.pos) * (iso - v0.value) / (v1.value - v0.value);
						interVal1 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value);
						interVal2 = v0.pos + (v2.pos - v0.pos) * (iso - v0.value) / (v2.value - v0.value);

						id0 = mesh.addVertex(interVal0, v0.index, v1.index);
						id1 = mesh.addVertex(interVal1, v0.index, v3.index);
						id2 = mesh.addVertex(interVal2, v0.index, v2.index);

						if (caseId == 1)
							mesh.addTriangle(id0, id1, id2);
						else
							mesh.addTriangle(id0, id2, id1); 
						*/
						break;

					case 2:
					case 13:
						/*
						interVal0 = v1.pos + (v0.pos - v1.pos) * (iso - v1.value) / (v0.value - v1.value);
						interVal1 = v1.pos + (v2.pos - v1.pos) * (iso - v1.value) / (v2.value - v1.value);
						interVal2 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value);

						id0 = mesh.addVertex(interVal0, v1.index, v0.index);
						id1 = mesh.addVertex(interVal1, v1.index, v2.index);
						id2 = mesh.addVertex(interVal2, v1.index, v3.index);

						if (caseId == 2)
							mesh.addTriangle(id0, id1, id2);
						else
							mesh.addTriangle(id0, id2, id1);

						break;*/

					case 3:
					case 12:
						
						interVal0 = v0.pos + (v2.pos - v0.pos) * (iso - v0.value) / (v2.value - v0.value);
						interVal1 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value);
						interVal2 = v1.pos + (v2.pos - v1.pos) * (iso - v1.value) / (v2.value - v1.value);
						interVal3 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value);


						id0 = mesh.addVertex(interVal0, v0.index, v2.index);
						id1 = mesh.addVertex(interVal1, v0.index, v3.index);
						id2 = mesh.addVertex(interVal2, v1.index, v2.index);
						id3 = mesh.addVertex(interVal3, v1.index, v3.index);

						if (caseId == 3)
						{
							mesh.addTriangle(id0, id2, id1);
							mesh.addTriangle(id1, id2, id3);
						}
						else
						{
							mesh.addTriangle(id0, id1, id2);
							mesh.addTriangle(id1, id3, id2);
						}
						
						break;
						
						


					case 4:
					case 11:
						/*
						interVal0 = v2.pos + (v0.pos - v2.pos) * (iso - v2.value) / (v0.value - v2.value);
						interVal1 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value);
						interVal2 = v2.pos + (v1.pos - v2.pos) * (iso - v2.value) / (v1.value - v2.value);

						id0 = mesh.addVertex(interVal0, v2.index, v0.index);
						id1 = mesh.addVertex(interVal1, v2.index, v3.index);
						id2 = mesh.addVertex(interVal2, v2.index, v1.index);

						if (caseId == 4)
							mesh.addTriangle(id0, id1, id2);
						else
							mesh.addTriangle(id0, id2, id1); */

						break;


					case 5:
					case 10:
						/*
						interVal0 = v0.pos + (v1.pos - v0.pos) * (iso - v0.value) / (v1.value - v0.value);
						interVal1 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value);
						interVal2 = v2.pos + (v1.pos - v2.pos) * (iso - v2.value) / (v1.value - v2.value);
						interVal3 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value);


						id0 = mesh.addVertex(interVal0, v0.index, v1.index);
						id1 = mesh.addVertex(interVal1, v0.index, v3.index);
						id2 = mesh.addVertex(interVal2, v2.index, v1.index);
						id3 = mesh.addVertex(interVal3, v2.index, v3.index);

						if (caseId == 5)
						{
							mesh.addTriangle(id0, id1, id2);
							mesh.addTriangle(id1, id3, id2);
						}
						else
						{
							mesh.addTriangle(id0, id2, id1);
							mesh.addTriangle(id1, id2, id3);
						}
						break;*/


					case 6:
					case 9:
						/*
						interVal0 = v1.pos + (v0.pos - v1.pos) * (iso - v1.value) / (v0.value - v1.value);
						interVal1 = v2.pos + (v0.pos - v2.pos) * (iso - v2.value) / (v0.value - v2.value);
						interVal2 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value);
						interVal3 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value);


						id0 = mesh.addVertex(interVal0, v1.index, v0.index);
						id1 = mesh.addVertex(interVal1, v2.index, v0.index);
						id2 = mesh.addVertex(interVal2, v1.index, v3.index);
						id3 = mesh.addVertex(interVal3, v2.index, v3.index);

						if (caseId == 6)
						{
							mesh.addTriangle(id0, id1, id2);
							mesh.addTriangle(id1, id3, id2);
						}
						else
						{
							mesh.addTriangle(id0, id2, id1);
							mesh.addTriangle(id1, id2, id3);
						}
						*/
						break;

					case 7:
					case 8:
						/*
						interVal0 = v0.pos + (v3.pos - v0.pos) * (iso - v0.value) / (v3.value - v0.value);
						interVal1 = v1.pos + (v3.pos - v1.pos) * (iso - v1.value) / (v3.value - v1.value);
						interVal2 = v2.pos + (v3.pos - v2.pos) * (iso - v2.value) / (v3.value - v2.value);

						id0 = mesh.addVertex(interVal0, v0.index, v3.index);
						id1 = mesh.addVertex(interVal1, v1.index, v3.index);
						id2 = mesh.addVertex(interVal2, v2.index, v3.index);

						if (caseId == 7)
							mesh.addTriangle(id0, id2, id1);
						else
							mesh.addTriangle(id0, id1, id2);
						*/
						break;

					}


                }
            }
        }
    }

    mesh_.setData(mesh.toBasicMesh());
}


MarchingTetrahedra::MeshHelper::MeshHelper(std::shared_ptr<const Volume> vol)
    : edgeToVertex_()
    , vertices_()
    , mesh_(std::make_shared<BasicMesh>())
    , indexBuffer_(mesh_->addIndexBuffer(DrawType::Triangles, ConnectivityType::None).get()) {
    mesh_->setModelMatrix(vol->getModelMatrix());
    mesh_->setWorldMatrix(vol->getWorldMatrix());
}

void MarchingTetrahedra::MeshHelper::addTriangle(size_t i0, size_t i1, size_t i2) {
    ivwAssert(i0 != i1, "i0 and i1 should not be the same value");
    ivwAssert(i0 != i2, "i0 and i2 should not be the same value");
    ivwAssert(i1 != i2, "i1 and i2 should not be the same value");

    indexBuffer_->add(static_cast<glm::uint32_t>(i0));
    indexBuffer_->add(static_cast<glm::uint32_t>(i1));
    indexBuffer_->add(static_cast<glm::uint32_t>(i2));

    auto a = std::get<0>(vertices_[i0]);
    auto b = std::get<0>(vertices_[i1]);
    auto c = std::get<0>(vertices_[i2]);

    vec3 n = glm::normalize(glm::cross(b - a, c - a));
    std::get<1>(vertices_[i0]) += n;
    std::get<1>(vertices_[i1]) += n;
    std::get<1>(vertices_[i2]) += n;
}

std::shared_ptr<BasicMesh> MarchingTetrahedra::MeshHelper::toBasicMesh() {
    for (auto& vertex : vertices_) {
        auto& normal = std::get<1>(vertex);
        normal = glm::normalize(normal);
    }
    mesh_->addVertices(vertices_);
    return mesh_;
}

std::uint32_t MarchingTetrahedra::MeshHelper::addVertex(vec3 pos, size_t i, size_t j) {
    ivwAssert(i != j, "i and j should not be the same value");
    if (j < i) {
        return addVertex(pos, j, i);
    }

    auto edge = std::make_pair(i, j);

    auto it = edgeToVertex_.find(edge);

    if (it == edgeToVertex_.end()) {
        edgeToVertex_[edge] = vertices_.size();
        vertices_.push_back({pos, vec3(0, 0, 0), pos, vec4(0.7f, 0.7f, 0.7f, 1.0f)});
        return static_cast<std::uint32_t>(vertices_.size() - 1);
    }

    return static_cast<std::uint32_t>(it->second);
}

}  // namespace inviwo
