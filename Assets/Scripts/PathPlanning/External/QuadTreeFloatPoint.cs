using System;
using System.Collections;
using System.Collections.Generic;

namespace QuadTreePoint
{
	#region · Types ·

	/// <summary>
	/// Interface for data class to be stored in the QuadTree
	/// </summary>
	public interface IQuadTreeData
	{
		/// <summary>Gets X coordinate of the point to be stored in the quadtree</summary>
		float X { get; }

		/// <summary>Gets Y coordinate of the point to be stored in the quadtree</summary>
		float Y { get; }
	}

	/// <summary>
	/// Class for storing rectangular (AABB) region. The region is stored as the center point and half size (width and height). The region must have the same width and height.
	/// </summary>
	public class QuadTreeFloatPointRegion
	{
		/// <summary>X coordinate of the center point of the region</summary>
		public float CenterX;

		/// <summary>Y coordinate of the center point of the region</summary>
		public float CenterY;

		/// <summary>Half size (Width and height) of the region</summary>
		public float HalfSize;

		/// <summary>
		/// Constructor from float values
		/// </summary>
		/// <param name="in_center_x">X coordinate of the center point of the region</param>
		/// <param name="in_center_y">Y coordinate of the center point of the region</param>
		/// <param name="in_half_size">Half size (Width and height) of the region</param>
		public QuadTreeFloatPointRegion(float in_center_x, float in_center_y, float in_half_size)
		{
			CenterX = in_center_x;
			CenterY = in_center_y;
			HalfSize = in_half_size;
		}

		/// <summary>
		/// Checks if the given quadtree data (point) is inside the region.
		/// </summary>
		/// <param name="in_point">Quadtree data containing the coordinates to check</param>
		/// <returns>True if point is inside the region</returns>
		public bool IsPointInside(IQuadTreeData in_point)
		{
			return (Math.Abs(CenterX - in_point.X) <= HalfSize) && (Math.Abs(CenterY - in_point.Y) <= HalfSize);
		}

		/// <summary>
		/// Checks if two regions are overlapping
		/// </summary>
		/// <param name="in_region">Other region to check</param>
		/// <returns>True if regions are overlapping</returns>
		public bool IsOverlapping(QuadTreeFloatPointRegion in_region)
		{
			return (Math.Abs(CenterX - in_region.CenterX) <= (in_region.HalfSize + HalfSize)) && (Math.Abs(CenterY - in_region.CenterY) <= (in_region.HalfSize + HalfSize));
		}

		/// <summary>
		/// Checks if two regions are overlapping. The other region is defined using it's left, top, width, height parameters
		/// </summary>
		/// <param name="in_left">Left X coordinate of the region</param>
		/// <param name="in_top">Top Y coordinate of the region</param>
		/// <param name="in_width">Width of the region</param>
		/// <param name="in_height">Height of the region</param>
		/// <returns></returns>
		public bool IsOverlapping(float in_left, float in_top, float in_width, float in_height)
		{
			return (CenterX - HalfSize < in_left + in_width) &&
							(CenterX + HalfSize > in_left) &&
							(CenterY - HalfSize < in_top + in_height) &&
							(CenterY + HalfSize > in_top);
		}

		/// <summary>
		/// Gets index of the quadrant where the given tree data (point) is located
		/// 
		/// The quadrant numbering:
		/// ---------
		/// | 0 | 1 |
		/// ---------
		/// | 2 | 3 |
		/// ---------
		/// </summary>
		/// <param name="in_point">Tree data (point) to check</param>
		/// <returns>Index of the quadrant [0..3]</returns>
		public int GetQuadrantIndex(IQuadTreeData in_point)
		{
			int quadrant = 0;

			if (in_point.X > CenterX)
				quadrant += 1;

			if (in_point.Y > CenterY)
				quadrant += 2;

			return quadrant;
		}

		/// <summary>
		/// Gets the square of the Euclidean distance of the specified point from the center of the region
		/// </summary>
		/// <param name="in_x">Point X coordinate</param>
		/// <param name="in_y">Point Y coordinate</param>
		/// <returns>Square of the Euclidean distance</returns>
		public double GetSquaredDistanceOfCenter(float in_x, float in_y)
		{
			return (CenterX - in_x) * (CenterX - in_x) + (CenterY - in_y) * (CenterY - in_y);
		}
	}

	#endregion

	/// <summary>
	/// Point Quad Tree class flor float type coordinates
	/// </summary>
	/// <typeparam name="T">Interface for the elements stored in the quadtee</typeparam>
	public class QuadTreeFloatPoint<T> : IEnumerable<T> where T : IQuadTreeData
	{
		#region · Types ·

		/// <summary>
		/// Leaf node in the quadtree. Contains items at the leaf in a linked list.
		/// </summary>
		private class QuadTreeLeaf
		{
			public T Data;
			public QuadTreeLeaf Next;

			public QuadTreeLeaf(T in_data)
			{
				Data = in_data;
				Next = null;
			}

			public double GetSquaredDistance(float in_x, float in_y)
			{
				return (Data.X - in_x) * (Data.X - in_x) + (Data.Y - in_y) * (Data.Y - in_y);
			}
		}

		/// <summary>
		/// Node of the quadtree. If it is an internal node in the tree it contans bounding box and four quadrants childen, if it is 
		/// </summary>
		private class QuadTreeNode
		{
			public QuadTreeFloatPointRegion Bounds;
			public QuadTreeNode[] Children;
			public QuadTreeLeaf Data;

			public QuadTreeNode(QuadTreeFloatPointRegion in_region)
			{
				Bounds = in_region;
				Children = null;
				Data = null;
			}
		}
		#endregion

		#region · Data members ·
		private QuadTreeNode m_root;
		private int m_bucket_capacity;
		private int m_node_count;
		#endregion

		#region · Constructor ·

		/// <summary>
		/// Construct QuadTree for the given region with the sepcified bucket capacity
		/// </summary>
		/// <param name="in_region">Coordinate region used for stored points</param>																					 
		/// <param name="in_bucket_capacity">Number of points stored in one bucket</param>
		public QuadTreeFloatPoint(QuadTreeFloatPointRegion in_region, int in_bucket_capacity)
		{
			m_root = new QuadTreeNode(in_region);
			m_bucket_capacity = in_bucket_capacity;
			m_node_count = 0;
		}

		/// <summary>
		/// Creates QuadTree for the specified region with bucket capacity set to one.
		/// </summary>
		/// <param name="in_region">Region  used for the stored point</param>
		public QuadTreeFloatPoint(QuadTreeFloatPointRegion in_region) : this(in_region, 1)
		{

		}


		/// <summary>
		/// Creates QuadTree from the given region specified by it's center and half width
		/// </summary>
		/// <param name="in_center_x">Center of the region (X coordinate)</param>
		/// <param name="in_center_y">Center of the region (Y coordinate)</param>
		/// <param name="in_half_size">Half size of the region</param>
		public QuadTreeFloatPoint(float in_center_x, float in_center_y, float in_half_size) : this(new QuadTreeFloatPointRegion(in_center_x, in_center_y, in_half_size))
		{
		}

		/// <summary>
		/// Creates QuadTree from the given region specified by it's center and half width using the soecified bucket capacity
		/// </summary>
		/// <param name="in_center_x"></param>
		/// <param name="in_center_y"></param>
		/// <param name="in_half_size"></param>
		/// <param name="in_bucket_size"></param>
		public QuadTreeFloatPoint(float in_center_x, float in_center_y, float in_half_size, int in_bucket_size) : this(new QuadTreeFloatPointRegion(in_center_x, in_center_y, in_half_size), in_bucket_size)
		{
		}

		#endregion

		#region · Tree building and maintenance ·

		/// <summary>
		/// Clears content of the quadtree
		/// </summary>
		public void Clear()
		{
			m_root.Children = null;
			m_root.Data = null;
		}

		/// <summary>
		/// Inserts a new node
		/// </summary>
		/// <param name="in_data"></param>
		public void Insert(T in_data)
		{
			QuadTreeLeaf node = new QuadTreeLeaf(in_data);

			Insert(m_root, node);
		}

		/// <summary>
		/// Inserts a new node
		/// </summary>
		/// <param name="in_data"></param>
		public bool Exists(T in_data)
		{
			QuadTreeLeaf node = new QuadTreeLeaf(in_data);

			return Exists(m_root, node);
		}

		/// <summary>
		/// Internal recursive insert function
		/// </summary>
		/// <param name="in_current_node"></param>
		/// <param name="in_node_to_insert"></param>
		private void Insert(QuadTreeNode in_current_node, QuadTreeLeaf in_node_to_insert)
		{
			// check if point ot insert is inside -> if it is not then it can not be child of this node
			if (!in_current_node.Bounds.IsPointInside(in_node_to_insert.Data))
				return;

			QuadTreeLeaf nodes_to_insert = null;
			QuadTreeLeaf node;
			int quadrant;

			// if this node is leaf
			if (in_current_node.Children == null)
			{
				// this is the first data item in the leaf
				if (in_current_node.Data == null)
				{
					in_current_node.Data = in_node_to_insert;

					return;
				}
				else
				{
					int item_count = 0;

					// add to the end of list of data
					node = in_current_node.Data;
					while (true)
					{
						if (Math.Abs(in_node_to_insert.Data.X - node.Data.X) <= float.Epsilon && Math.Abs(in_node_to_insert.Data.Y - node.Data.Y) <= float.Epsilon)
							throw new ArgumentException("Key already exists");

						item_count++;

						if (node.Next != null)
							node = node.Next;
						else
							break;
					}

					// there is room for this item
					if (item_count < m_bucket_capacity)
					{
						// add node to the list of data
						node.Next = in_node_to_insert;

						return;
					}

					// current node needs to be splitted
					nodes_to_insert = (QuadTreeLeaf)in_current_node.Data;

					// remove data
					in_current_node.Data = null;
				}
			}
			else
			{
				// move downward on the tree following the apropriate quadrant
				quadrant = in_current_node.Bounds.GetQuadrantIndex(in_node_to_insert.Data);

				Insert(in_current_node.Children[quadrant], in_node_to_insert);

				return;
			}

			// subdivide current node
			QuadTreeFloatPointRegion bounds = in_current_node.Bounds;
			float half = bounds.HalfSize / 2;
			QuadTreeNode[] subdivision = new QuadTreeNode[4] {
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX - half, bounds.CenterY - half, half)),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX + half, bounds.CenterY - half, half )),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX - half, bounds.CenterY + half, half )),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX + half, bounds.CenterY + half, half ))
			};

			// insert node
			quadrant = bounds.GetQuadrantIndex(in_node_to_insert.Data);

			Insert(subdivision[quadrant], in_node_to_insert);

			// insert nodes from the splitted node
			node = nodes_to_insert;
			QuadTreeLeaf next_node;
			while (node != null)
			{
				next_node = node.Next;
				node.Next = null;

				quadrant = bounds.GetQuadrantIndex(node.Data);

				Insert(subdivision[quadrant], node);

				node = next_node;
			}

			in_current_node.Children = subdivision;

		}

		/// <summary>
		/// Internal recursive insert function
		/// </summary>
		/// <param name="in_current_node"></param>
		/// <param name="in_node_to_insert"></param>
		private bool Exists(QuadTreeNode in_current_node, QuadTreeLeaf in_node_to_insert)
		{
			// check if point ot insert is inside -> if it is not then it can not be child of this node
			if (!in_current_node.Bounds.IsPointInside(in_node_to_insert.Data))
				return false;
            QuadTreeLeaf node;
            int quadrant;

            // if this node is leaf
            if (in_current_node.Children == null)
			{
				// this is the first data item in the leaf
				if (in_current_node.Data == null)
				{
					//in_current_node.Data = in_node_to_insert;

					return false;
				}
				else
				{
					int item_count = 0;

					// add to the end of list of data
					node = in_current_node.Data;
					while (true)
					{
						if (Math.Abs(in_node_to_insert.Data.X - node.Data.X) <= float.Epsilon && Math.Abs(in_node_to_insert.Data.Y - node.Data.Y) <= float.Epsilon)
							return true;

						item_count++;

						if (node.Next != null)
							node = node.Next;
						else
							break;
					}

					// there is room for this item
					if (item_count < m_bucket_capacity)
					{
						// add node to the list of data
						//node.Next = in_node_to_insert;

						return false;
					}

					// current node needs to be splitted
					//nodes_to_insert = (QuadTreeLeaf)in_current_node.Data;

					// remove data
					//in_current_node.Data = null;
				}
			}
			else
			{
				// move downward on the tree following the apropriate quadrant
				quadrant = in_current_node.Bounds.GetQuadrantIndex(in_node_to_insert.Data);

				return Exists(in_current_node.Children[quadrant], in_node_to_insert);
			}
			return false;
			/*
			// subdivide current node
			QuadTreeFloatPointRegion bounds = in_current_node.Bounds;
			float half = bounds.HalfSize / 2;
			QuadTreeNode[] subdivision = new QuadTreeNode[4] {
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX - half, bounds.CenterY - half, half)),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX + half, bounds.CenterY - half, half )),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX - half, bounds.CenterY + half, half )),
				new QuadTreeNode(new QuadTreeFloatPointRegion(bounds.CenterX + half, bounds.CenterY + half, half ))
			};

			// insert node
			quadrant = bounds.GetQuadrantIndex(in_node_to_insert.Data);

			Insert(subdivision[quadrant], in_node_to_insert);

			// insert nodes from the splitted node
			node = nodes_to_insert;
			QuadTreeLeaf next_node;
			while (node != null)
			{
				next_node = node.Next;
				node.Next = null;

				quadrant = bounds.GetQuadrantIndex(node.Data);

				Insert(subdivision[quadrant], node);

				node = next_node;
			}

			in_current_node.Children = subdivision;*/

		}

		#endregion

		#region · Traversing, enumerating ·

		public delegate void NodeProcessCallback(QuadTreeFloatPointRegion in_bounds);
		public delegate void DataProcessCallback(T in_data);

		public void TraverseNodesAndLeafs(DataProcessCallback in_data_process_callback, NodeProcessCallback in_node_process_callback)
		{
			Stack<QuadTreeNode> stack = new Stack<QuadTreeNode>();
			QuadTreeNode current = m_root;

			while (true)
			{
				if (current.Children != null)
				{
					if (in_node_process_callback != null)
					{
						foreach (QuadTreeNode node in current.Children)
							in_node_process_callback(node.Bounds);
					}

					stack.Push(current.Children[2]);
					stack.Push(current.Children[1]);
					stack.Push(current.Children[3]);
					current = current.Children[0];
				}
				else
				{
					QuadTreeLeaf node = current.Data;

					while (node != null)
					{
						in_data_process_callback(node.Data);


						node = node.Next;
					}

					if (stack.Count > 0)
						current = stack.Pop();
					else
						break;
				}
			}
		}

		public IEnumerable<T> Query(float in_left, float in_top, float in_width, float in_height)
		{
			Stack<QuadTreeNode> stack = new Stack<QuadTreeNode>();
			QuadTreeNode current = m_root;

			while (current != null)
			{
				if (current.Children != null)
				{
					QuadTreeNode[] children = current.Children;

					if (children[2].Bounds != null && children[2].Bounds.IsOverlapping(in_left, in_top, in_width, in_height))
						stack.Push(children[2]);

					if (children[1].Bounds != null && children[1].Bounds.IsOverlapping(in_left, in_top, in_width, in_height))
						stack.Push(children[1]);

					if (children[3].Bounds != null && children[3].Bounds.IsOverlapping(in_left, in_top, in_width, in_height))
						stack.Push(children[3]);

					if (children[0].Bounds != null && children[0].Bounds.IsOverlapping(in_left, in_top, in_width, in_height))
						stack.Push(children[0]);
				}
				else
				{
					QuadTreeLeaf node = current.Data;

					while (node != null)
					{
						if (node.Data.X > in_left && node.Data.X < in_left + in_width && node.Data.Y > in_top && node.Data.Y < in_top + in_height)
							yield return node.Data;

						node = node.Next;
					}
				}

				if (stack.Count > 0)
					current = stack.Pop();
				else
					break;
			}
		}

		public IEnumerator<T> GetEnumerator()
		{
			Stack<QuadTreeNode> stack = new Stack<QuadTreeNode>();
			QuadTreeNode current = m_root;

			while (current != null)
			{
				if (current.Children != null)
				{
					stack.Push(current.Children[2]);
					stack.Push(current.Children[1]);
					stack.Push(current.Children[3]);
					stack.Push(current.Children[0]);
				}
				else
				{
					QuadTreeLeaf node = current.Data;

					while (node != null)
					{
						yield return node.Data;

						node = node.Next;
					}
				}

				if (stack.Count > 0)
					current = stack.Pop();
				else
					break;
			}
		}

		IEnumerator IEnumerable.GetEnumerator()
		{
			return GetEnumerator();
		}

		#endregion

		#region · Nearest neighbour search ·

		public List<T> QueryNeighbours(float in_x, float in_y, int in_neighbours_count)
		{
			QuadTreeFloatPointRegion neighbour_region = new QuadTreeFloatPointRegion(in_x, in_y, 0);
			Stack<QuadTreeNode> stack = new Stack<QuadTreeNode>();
			QuadTreeNode current;

			List<T> neighbours = new List<T>();
			double[] neighbour_distances = new double[in_neighbours_count];
			double neighbours_worst_distance = 0;
			int neighbours_worst_index = 0;

			// set root node as current
			current = m_root;

			while (current != null)
			{

				// move downwards if this node has child nodes
				if (current.Children != null)
				{
					// store regions in the stack and continue with the closest region
					double closest_region_distance;
					int closest_region_index;

					// find closest region
					closest_region_index = 0;
					closest_region_distance = current.Children[0].Bounds.GetSquaredDistanceOfCenter(in_x, in_y);

					for (int i = 0; i < 4; i++)
					{
						double distance = current.Children[i].Bounds.GetSquaredDistanceOfCenter(in_x, in_y);
						if (distance < closest_region_distance)
						{
							closest_region_distance = distance;
							closest_region_index = i;
						}
					}

					// store regions
					for (int i = 0; i < 4; i++)
					{
						if (i == closest_region_index)
							continue;

						// if the neighbor region is defined then store only the overlapping regions, otherwise store all regions
						if (neighbour_region.HalfSize == 0 || current.Children[i].Bounds.IsOverlapping(neighbour_region))
							stack.Push(current.Children[i]);
					}

					// continue processing with the closest	region
					current = current.Children[closest_region_index];
				}
				else
				{
					// process data points
					QuadTreeLeaf current_leaf_entry = current.Data;

					while (current_leaf_entry != null)
					{
						// calculate distance (squared)
						double squared_distance = current_leaf_entry.GetSquaredDistance(in_x, in_y);

						if (current.Data != null)
						{
							// simply store data point if the list is not full
							if (neighbours.Count < in_neighbours_count)
							{
								if (neighbours.Count == 0)
								{
									neighbours_worst_distance = squared_distance;
									neighbours_worst_index = 0;
								}
								else
								{
									if (squared_distance > neighbours_worst_distance)
									{
										neighbours_worst_distance = squared_distance;
										neighbours_worst_index = neighbours.Count;
									}
								}

								// add this item to the neighbours list
								neighbour_distances[neighbours.Count] = squared_distance;
								neighbours.Add(current_leaf_entry.Data);

								// if the required number of neighbour is found store the worst distance in the region
								if (neighbours.Count == in_neighbours_count)
								{
									neighbour_region.HalfSize = (float)Math.Sqrt(neighbours_worst_distance);
								}
							}
							else
							{
								// list is full, store only when this item is closer than the worst item (largest distance) in the list
								if (squared_distance < neighbours_worst_distance)
								{
									// replace worst element
									neighbour_distances[neighbours_worst_index] = squared_distance;
									neighbours[neighbours_worst_index] = current_leaf_entry.Data;

									// find the current worst element
									neighbours_worst_index = 0;
									neighbours_worst_distance = neighbour_distances[0];
									for (int i = 1; i < in_neighbours_count; i++)
									{
										if (neighbour_distances[i] > neighbours_worst_distance)
										{
											neighbours_worst_distance = neighbour_distances[i];
											neighbours_worst_index = i;
										}
									}

									neighbour_region.HalfSize = (float)Math.Sqrt(neighbours_worst_distance);
								}
							}
						}

						current_leaf_entry = current_leaf_entry.Next;
					}

					// get new element from the stack or exit if no more element to investigate
					do
					{
						if (stack.Count > 0)
						{
							current = stack.Pop();
						}
						else
						{
							current = null;
							break;
						}

						// if the neighbour region is know skip all elements with a non-overlapping region
					} while (neighbour_region.HalfSize > 0 && !current.Bounds.IsOverlapping(neighbour_region));


				}
			}

			return neighbours;
		}

		#endregion
	}
}