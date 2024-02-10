using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Rendering;

namespace util
{
    class MeshUtil
    {
        public static List<Rect> BoundingBoxesFromMesh(Mesh mesh, Transform transform, float margin, float yMin, float yMax, float similarityThres = 0.1f)
        {

            List<Vector3>[] vertexGroups = FindConnectedVertices(mesh, transform, yMin, yMax, similarityThres);

            // Construct bounding boxes
            var boundingBoxes = new List<Rect>();
            foreach (List<Vector3> vertices in vertexGroups)
            {
                float xMin = float.MaxValue;
                float xMax = float.MinValue;
                float zMin = float.MaxValue;
                float zMax = float.MinValue;

                foreach (Vector3 v in vertices)
                {
                    if (v.x < xMin) xMin = v.x;
                    if (v.x > xMax) xMax = v.x;
                    if (v.z < zMin) zMin = v.z;
                    if (v.z > zMax) zMax = v.z;
                }
                Rect rect = new Rect();
                rect.xMin = xMin - margin;
                rect.xMax = xMax + margin;
                rect.yMin = zMin - margin;
                rect.yMax = zMax + margin;

                boundingBoxes.Add(rect);
            }

            return boundingBoxes;
        }
        static List<Vector3>[] FindConnectedVertices(Mesh mesh, Transform transform, float yMin, float yMax, float similarityThres)
        {
            int[] indices = mesh.triangles;

            // Transform vertices to world coordinates
            Vector3[] vertices = new Vector3[mesh.vertices.Length];
            for (int i = 0; i < mesh.vertices.Length; i++)
            {
                vertices[i] = transform.TransformPoint(mesh.vertices[i]);
            }

            // Find groups of connected vertices
            var groupIdDict = new Dictionary<Vector3Int, int>();
            for (int i = 0; i < indices.Length; i += 3)
            {

                // Find minimum groupId and cull triangles outside y-limits
                bool skipTriangle = false;
                int minGroupId = -1;
                for (int j = 0; j < 3; j++)
                {
                    int index = indices[i + j];

                    // Cull triangles
                    if (vertices[index].y < yMin || vertices[index].y > yMax)
                    {
                        skipTriangle = true;
                        break;
                    }

                    // Minimum groupId
                    var vertexKey = GetVertexKey(vertices[index], similarityThres);
                    if (groupIdDict.TryGetValue(vertexKey, out int groupId))
                    {
                        if (groupId < minGroupId || minGroupId == -1)
                            minGroupId = groupId;
                    }
                }
                if (skipTriangle) continue;

                // Assign new groupId to vertices
                if (minGroupId == -1)
                {
                    // Create new groupId if it doesn't exist
                    int newGroupId = indices[i];
                    for (int j = 0; j < 3; j++)
                    {
                        int index = indices[i + j];
                        var vertexKey = GetVertexKey(vertices[index], similarityThres);
                        groupIdDict[vertexKey] = newGroupId;
                    }
                }
                else
                {
                    // Set all connected vertices to the same groupId
                    int lastUpdated = minGroupId;
                    for (int j = 0; j < 3; j++)
                    {
                        int index = indices[i + j];
                        var vertexKey = GetVertexKey(vertices[index], similarityThres);
                        if (groupIdDict.TryGetValue(vertexKey, out int groupId))
                        {
                            if (groupId != minGroupId)
                            {
                                groupIdDict[vertexKey] = minGroupId;

                                if (lastUpdated == groupId) // Small optimization, don't update the same values twice 
                                    continue;

                                // Update connected indices to same groupId
                                foreach (var key in groupIdDict.Keys.ToArray())
                                {
                                    if (groupIdDict[key] == groupId)
                                        groupIdDict[key] = minGroupId;
                                }
                                lastUpdated = groupId;
                            }
                        }
                        else
                        {
                            groupIdDict[vertexKey] = minGroupId;
                        }
                    }
                }
            }

            // Format data into separate groups
            var vertexListDict = new Dictionary<int, List<Vector3>>();
            for (int i = 0; i < indices.Length; i++)
            {
                int index = indices[i];
                var vertexKey = GetVertexKey(vertices[index], similarityThres);
                if (!groupIdDict.ContainsKey(vertexKey))
                    continue;

                int groupId = groupIdDict[vertexKey];
                if (!vertexListDict.ContainsKey(groupId))
                    vertexListDict[groupId] = new List<Vector3>();

                vertexListDict[groupId].Add(vertices[index]);
            }

            // Sort into final array
            int nGroups = vertexListDict.Keys.Count;
            var vertexListArray = new List<Vector3>[nGroups];
            int arrayIndex = 0;
            foreach (int groupId in vertexListDict.Keys)
            {
                vertexListArray[arrayIndex] = vertexListDict[groupId];
                arrayIndex++;
            }

            return vertexListArray;
        }
        static Vector3Int GetVertexKey(Vector3 vertex, float similarityThres)
        {
            return Vector3Int.RoundToInt(vertex * (1f / similarityThres));
        }

        // Needed to get vertex data on CPU, taken from https://forum.unity.com/threads/reading-meshes-at-runtime-that-are-not-enabled-for-read-write.950170/
        public static Mesh MakeReadableMeshCopy(Mesh nonReadableMesh)
        {
            Mesh meshCopy = new Mesh();
            meshCopy.indexFormat = nonReadableMesh.indexFormat;

            // Handle vertices
            GraphicsBuffer verticesBuffer = nonReadableMesh.GetVertexBuffer(0);
            int totalSize = verticesBuffer.stride * verticesBuffer.count;
            byte[] data = new byte[totalSize];
            verticesBuffer.GetData(data);
            meshCopy.SetVertexBufferParams(nonReadableMesh.vertexCount, nonReadableMesh.GetVertexAttributes());
            meshCopy.SetVertexBufferData(data, 0, 0, totalSize);
            verticesBuffer.Release();

            // Handle triangles
            meshCopy.subMeshCount = nonReadableMesh.subMeshCount;
            GraphicsBuffer indexesBuffer = nonReadableMesh.GetIndexBuffer();
            int tot = indexesBuffer.stride * indexesBuffer.count;
            byte[] indexesData = new byte[tot];
            indexesBuffer.GetData(indexesData);
            meshCopy.SetIndexBufferParams(indexesBuffer.count, nonReadableMesh.indexFormat);
            meshCopy.SetIndexBufferData(indexesData, 0, 0, tot);
            indexesBuffer.Release();

            // Restore submesh structure
            uint currentIndexOffset = 0;
            for (int i = 0; i < meshCopy.subMeshCount; i++)
            {
                uint subMeshIndexCount = nonReadableMesh.GetIndexCount(i);
                meshCopy.SetSubMesh(i, new SubMeshDescriptor((int)currentIndexOffset, (int)subMeshIndexCount));
                currentIndexOffset += subMeshIndexCount;
            }

            // Recalculate normals and bounds
            meshCopy.RecalculateNormals();
            meshCopy.RecalculateBounds();

            return meshCopy;
        }
    }
}
