
using System;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.Triangulation.Data;
using Pathfinding.Triangulation.Iterators;
using Pathfinding.Triangulation.Math;
using Pathfinding.Util;

namespace Pathfinding.Triangulation.AI
{
    public enum EFindPathResult
    {
        Success = 0,
        Failed,
        Progress
    }
    public class AStar
    {
        public AStar()
        {
            iterEdge = new FromFaceToInnerEdges();
        }

        private FixMath.F64 _radius;

        private Mesh _mesh;

        private FromFaceToInnerEdges iterEdge;
        private FixMath.F64 radiusSquared;
        private FixMath.F64 diameter;
        private FixMath.F64 diameterSquared;
        private Face fromFace;
        private Face toFace;

        public void dispose()
        {
            this._mesh = null;
        }


        public FixMath.F64 get_radius()
        {
            return this._radius;
        }


        public FixMath.F64 set_radius(FixMath.F64 rdius)
        {
            this._radius = rdius;
            this.radiusSquared = (this._radius * this._radius);
            this.diameter = (this._radius * 2);
            this.diameterSquared = (this.diameter * this.diameter);
            return rdius;
        }

        public Mesh set_mesh(Mesh mesh)
        {
            this._mesh = mesh;
            return mesh;
        }

        enum NodeSearchState
        {
            NODE_STATE_OPEN = 0,
            NODE_STATE_CLOSE = 1,
            NODE_STATE_SEQUENCE = 2,
        };

        public class SearchNode
        {
            Face face = null;
            public Edge EntryEdge { get; set; }
            public FixMath.F64Vec2 EntryPoint { get; set; }
            // path search
            int ArrayIndexInHeap = -1;
            FixMath.F64 FCost = FixMath.F64.Zero, GCost = FixMath.F64.Zero, HCost = FixMath.F64.Zero;
            SearchNode PrevNode = null;
            // offset that identifies nodes as part of current search
            int SearchState = 0;    // seq | (open/close)

            public bool IsOpen() { return (SearchState & 1) == 0; }
            public bool IsClose() { return (SearchState & 1) == 1; }
            public void SetFace(Face f) { face = f; }
            public Face GetFace() { return face; }
            public void SetHeapIndex(int idx) { ArrayIndexInHeap = idx; }
            public int GetHeapIndex() { return ArrayIndexInHeap; }
            public FixMath.F64 GetFCost() { return FCost; }
            public void SetFCost(FixMath.F64 c) { FCost = c; }
            public FixMath.F64 GetGCost() { return GCost; }
            public void SetGCost(FixMath.F64 c) { GCost = c; }
            public FixMath.F64 GetHCost() { return HCost; }
            public void SetHCost(FixMath.F64 c) { HCost = c; }
            public void SetPrevNode(SearchNode n) { PrevNode = n; }
            public SearchNode GetPrevNode() { return PrevNode; }

            public void ClearSearchState()
            {
                ArrayIndexInHeap = -1;
                PrevNode = null;
            }

            public void SetSearchState(int state) { SearchState = state; }
            public int GetSearchState() { return SearchState; }
        }

        class SearchHeap
        {
            public void Reset()
            {
                HeapNodes.Clear();
            }

            public void Push(SearchNode InNode)
            {
                HeapNodes.Add(InNode);
                int CurIndex = HeapNodes.Count - 1;
                InNode.SetHeapIndex(CurIndex);

                int ParentIndex = -1;
                SearchNode TmpNode = null;
                while (CurIndex > 0)
                {
                    ParentIndex = CurIndex >> 1;

                    if (HeapNodes[CurIndex].GetFCost() < HeapNodes[ParentIndex].GetFCost())
                    {
                        TmpNode = HeapNodes[CurIndex];
                        HeapNodes[CurIndex] = HeapNodes[ParentIndex];
                        HeapNodes[ParentIndex] = TmpNode;

                        HeapNodes[ParentIndex].SetHeapIndex(ParentIndex);
                        HeapNodes[CurIndex].SetHeapIndex(CurIndex);

                        CurIndex = ParentIndex;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            public SearchNode Pop()
            {
                SearchNode MinNode = HeapNodes[0];

                int LastIndex = HeapNodes.Count - 1;
                HeapNodes[0] = HeapNodes[LastIndex];
                HeapNodes[0].SetHeapIndex(0);
                HeapNodes.RemoveAt(LastIndex);

                int CurIndex = 0;
                int CurSize = HeapNodes.Count;
                int LeftChild = CurIndex * 2 + 1;
                int RightChild = LeftChild + 1;
                int MinChildIndex = -1;
                SearchNode TmpNode = null;
                while (LeftChild < CurSize)
                {
                    if (RightChild < CurSize)
                    {
                        MinChildIndex = (HeapNodes[RightChild].GetFCost() < HeapNodes[LeftChild].GetFCost()) ? RightChild : LeftChild;
                    }
                    else
                    {
                        MinChildIndex = LeftChild;
                    }
                    if (HeapNodes[MinChildIndex].GetFCost() < HeapNodes[CurIndex].GetFCost())
                    {
                        TmpNode = HeapNodes[MinChildIndex];
                        HeapNodes[MinChildIndex] = HeapNodes[CurIndex];
                        HeapNodes[CurIndex] = TmpNode;

                        HeapNodes[CurIndex].SetHeapIndex(CurIndex);
                        HeapNodes[MinChildIndex].SetHeapIndex(MinChildIndex);

                        CurIndex = MinChildIndex;

                        LeftChild = CurIndex * 2 + 1;
                        RightChild = LeftChild + 1;
                    }
                    else
                    {
                        break;
                    }
                }
                return MinNode;
            }

            public void Update(SearchNode InNode)
            {
                int CurIndex = InNode.GetHeapIndex();

                int ParentIndex = -1;
                SearchNode TmpNode = null;
                while (CurIndex > 0)
                {
                    ParentIndex = CurIndex >> 1;

                    if (HeapNodes[CurIndex].GetFCost() < HeapNodes[ParentIndex].GetFCost())
                    {
                        TmpNode = HeapNodes[CurIndex];
                        HeapNodes[CurIndex] = HeapNodes[ParentIndex];
                        HeapNodes[ParentIndex] = TmpNode;

                        HeapNodes[ParentIndex].SetHeapIndex(ParentIndex);
                        HeapNodes[CurIndex].SetHeapIndex(CurIndex);
                        CurIndex = ParentIndex;
                    }
                    else
                    {
                        break;
                    }
                }
            }

            public bool IsEmpty()
            {
                return HeapNodes.Count <= 0;
            }

            public int Size()
            {
                return HeapNodes.Count;
            }

            private List<SearchNode> HeapNodes = new List<SearchNode>();
        }

        private List<SearchNode> UsedNodeList = new List<SearchNode>();
        private Dictionary<Face, SearchNode> SearchNodeMap = new Dictionary<Face, SearchNode>();
        private Stack SearchNodeCache = new Stack();
        private int SearchState = 0;
        private int MaxSearchNodeNum = 1024;
        private int CurrSearchNodeNum = 0;
        private SearchNode StartNode = null;
        private SearchNode GoalNode = null;
        private SearchNode MinNode = null;
        private FixMath.F64Vec2 StartPoint = FixMath.F64Vec2.Zero;
        private FixMath.F64Vec2 GoalPoint = FixMath.F64Vec2.Zero;

        private SearchHeap SearchMinHeap = new SearchHeap();

        // 归还所有当前在使用的Node
        void freeAllSearchNode()
        {
            for (int i = 0; i < UsedNodeList.Count; ++i)
            {
                SearchNodeCache.Push(UsedNodeList[i]);
            }

            UsedNodeList.Clear();
            SearchNodeMap.Clear();
        }

        SearchNode findOrCreateSearchNode(Face face)
        {
            // 优先在HashTable中查找
            SearchNode node = null;
            if (SearchNodeMap.TryGetValue(face, out node))
            { 
                return node;
            }
            else
            {
                // 检查Cache中是否有空闲的Node
                if (SearchNodeCache.Count > 0)
                {
                    node = SearchNodeCache.Pop() as SearchNode;
                }
                else
                {
                    // 分配新的节点
                    node = new SearchNode();
                }

                node.SetFCost(FixMath.F64.Zero);
                node.SetGCost(FixMath.F64.Zero);
                node.SetHCost(FixMath.F64.Zero);
                node.SetPrevNode(null);
                node.SetHeapIndex(-1);
                node.SetFace(face);

                UsedNodeList.Add(node);
                SearchNodeMap.Add(face, node);
            }
            return node;
        }

        void updateNode(
            SearchNode InNextNode,
            SearchNode InPrevNode,
            FixMath.F64 newG,
            FixMath.F64 newH)
        {
            InNextNode.SetPrevNode(InPrevNode);
            InNextNode.SetHCost(newH);
            InNextNode.SetGCost(newG);
            InNextNode.SetFCost(newG + newH);
            InNextNode.SetSearchState(SearchState | (int)NodeSearchState.NODE_STATE_OPEN);
        }

        SearchNode IterateNodes()
        {
            var CurNode = SearchMinHeap.Pop();
            CurNode.SetSearchState(SearchState | (int)NodeSearchState.NODE_STATE_CLOSE);

            if (CurNode == GoalNode)
            {
                MinNode = CurNode;
                return CurNode;
            }

            // 检查是否超过最大搜索节点数目
            if (CurrSearchNodeNum >= MaxSearchNodeNum)
            {
                return CurNode;
            }

            if (CurNode.GetHCost() < MinNode.GetHCost())
            {
                MinNode = CurNode;
            }

            IterateNodeNeighbors(CurNode);

            return CurNode;
        }

        void IterateNodeNeighbors(SearchNode InCurNode)
        {
            Edge innerEdge;
            Face neighbourFace;

            Face curFace = InCurNode.GetFace();
            iterEdge.set_fromFace(curFace);
            while ((innerEdge = iterEdge.next()) != null)
            {
                if (innerEdge._isConstrained)
                    continue;
                neighbourFace = innerEdge.get_rightFace();

                if (curFace != fromFace && _radius > 0 && !isWalkableByRadius(InCurNode.EntryEdge, curFace, innerEdge))
                {
                    //                            Debug.trace("- NOT WALKABLE -");
                    //                            Debug.trace( "from ", hxDaedalusEdge(__entryEdges[__curFace]).originVertex.id, hxDaedalusEdge(__entryEdges[__curFace]).destinationVertex.id );
                    //                            Debug.trace( "to", innerEdge.originVertex.id, innerEdge.destinationVertex.id );
                    //                            Debug.trace("----------------");
                    continue;
                }

                var NextNode = findOrCreateSearchNode(neighbourFace);

                bool IsCurrent = NextNode.GetSearchState() >= SearchState;
                bool IsClosed = NextNode.IsClose();
                bool IsTarget = NextNode == GoalNode;

                // If we found an unexplored node, add it to the open list.
                // Otherwise, if the new cost is lower, refresh the cost of the node 
                // as we just found a shorter path.
                var midPoint = FixMath.F64Vec2.Zero;
                midPoint.X = (innerEdge.get_originVertex()._pos.X + innerEdge.get_destinationVertex()._pos.X) / 2;
                midPoint.Y = (innerEdge.get_originVertex()._pos.Y + innerEdge.get_destinationVertex()._pos.Y) / 2;

                var distancePoint = InCurNode.EntryPoint - midPoint;
                var newG = InCurNode.GetGCost() + FixMath.F64Vec2.LengthFast(distancePoint);

                var newH = FixMath.F64.Zero;
                if (!IsTarget)
                {
                    distancePoint = GoalPoint - midPoint;
                    newH = FixMath.F64Vec2.LengthFast(distancePoint);
                }

                var fillDatas = false;
                if (!IsCurrent)
                {
                    fillDatas = true;
                    updateNode(NextNode, InCurNode, newG, newH);
                    SearchMinHeap.Push(NextNode);
                    ++CurrSearchNodeNum;
                }
                else if (NextNode.IsOpen())
                {
                    var NewF = newG + newH;
                    if (NewF < NextNode.GetFCost())
                    {
                        fillDatas = true;
                        updateNode(NextNode, InCurNode, newG, newH);
                    }
                }

                if (fillDatas)
                {
                    NextNode.EntryEdge = innerEdge;
                    NextNode.EntryPoint = midPoint;
                }
            }
        }

        public void findPath(FixMath.F64 fromX, FixMath.F64 fromY
                            , FixMath.F64 toX, FixMath.F64 toY
                            , int maxSearchNodes     // 最大搜索节点数
                            , List<Face> resultListFaces
                            , List<Edge> resultListEdges
                            , out bool isPartial)
        {
            //Debug.trace("findPath");
            isPartial = false;

            Intersection loc;
            Edge locEdge;
            Vertex locVertex;
            FixMath.F64 distance;
            FixMath.F64Vec2 p1;
            FixMath.F64Vec2 p2;
            FixMath.F64Vec2 p3;
            //
            loc = Geom2D.locatePosition(fromX, fromY, _mesh);
            {
                if (loc is Intersection_EVertex v0)
                {
                    locVertex = v0.vertex;
                    return;
                }
                else if (loc is Intersection_EEdge e0)
                {
                    locEdge = e0.edge;
                    return;
                }
                else if (loc is Intersection_EFace f0)
                {
                    fromFace = f0.face;
                }
            }


            loc = Geom2D.locatePosition(toX, toY, _mesh);
            {
                if (loc is Intersection_EVertex v1)
                {
                    locVertex = v1.vertex;
                    toFace = locVertex._edge.get_leftFace();
                }
                else if (loc is Intersection_EEdge e1)
                {
                    locEdge = e1.edge;
                    toFace = locEdge.get_leftFace();
                }
                else if (loc is Intersection_EFace f1)
                {
                    toFace = f1.face;
                }
            }


            // 清理上一次使用的所有节点
            freeAllSearchNode();

            CurrSearchNodeNum = 0;
            MaxSearchNodeNum = maxSearchNodes;
            SearchMinHeap.Reset();
            SearchState += (int)NodeSearchState.NODE_STATE_SEQUENCE;

            this.StartNode = findOrCreateSearchNode(fromFace);
            this.GoalNode = findOrCreateSearchNode(toFace);
            if (StartNode == null || GoalNode == null)
            {
                return;
            }

            StartNode.EntryEdge = null;
            StartNode.EntryPoint = new FixMath.F64Vec2(fromX, fromY);

            var dist = FixMath.F64.Sqrt((toX - fromX) * (toX - fromX) + (toY - fromY) * (toY - fromY));
            // 压入开始Node到Heap中，并初始化
            StartNode.SetHCost(dist);
            StartNode.SetFCost(dist);
            StartNode.SetSearchState(SearchState | (int)NodeSearchState.NODE_STATE_OPEN);
            SearchMinHeap.Push(StartNode);

            // 初始化当前最小的节点
            MinNode = StartNode;
            StartPoint = new FixMath.F64Vec2(fromX, fromY);
            GoalPoint = new FixMath.F64Vec2(toX, toY);

            var HasFullPath = false;
            var HasPartPath = false;
            var LoopCount = 0;
            const int MaxLoopCount = 5000;
            while (!SearchMinHeap.IsEmpty())
            {
                if (++LoopCount >= MaxLoopCount)
                {
                    Debug.Assert(false, string.Format("FindPath LoopCount:{0}", LoopCount));
                    break;
                }

                if (CurrSearchNodeNum >= MaxSearchNodeNum)
                {
                    break;
                }

                var CurNode = IterateNodes();

                HasFullPath = CurNode == GoalNode;
                HasPartPath = MinNode != StartNode;

                if (HasFullPath)
                {
                    break;
                }
            }

            if (MinNode == null)
                return;  // else we build the path  ;
            isPartial = MinNode.GetFace() != toFace;

            var curNode = MinNode;
            resultListFaces.Add(curNode.GetFace());
            while (curNode.GetFace() != fromFace)
            {
                resultListEdges.Insert(0, curNode.EntryEdge);
                resultListFaces.Insert(0, curNode.GetFace());
                curNode = curNode.GetPrevNode();
            }
        }

        public EFindPathResult initialPath(FixMath.F64 fromX, FixMath.F64 fromY
                                    , FixMath.F64 toX, FixMath.F64 toY)
        {
            return EFindPathResult.Failed;
        }

        public EFindPathResult updatePath(int stepNum
                            , List<Face> resultListFaces
                            , List<Edge> resultListEdges)
        {
            return EFindPathResult.Failed;
        }

        //public void findPath(FixMath.F64 fromX, FixMath.F64 fromY
        //                    , FixMath.F64 toX, FixMath.F64 toY
        //                    , int iterMaxTimes     // 最大迭代次数
        //                    , List<Face> resultListFaces
        //                    , List<Edge> resultListEdges
        //                    , out bool isPartial) 
        //{
        //    //Debug.trace("findPath");
        //    isPartial = false;
        //    closedFaces = new Dictionary<Face, bool>();
        //    sortedOpenedFaces = new List<Face>();
        //    openedFaces = new Dictionary<Face, bool>();
        //    entryEdges = new Dictionary<Face, Edge>();
        //    entryX = new Dictionary<Face, FixMath.F64>();
        //    entryY = new Dictionary<Face, FixMath.F64>();
        //    scoreF = new Dictionary<Face, FixMath.F64>();
        //    scoreG = new Dictionary<Face, FixMath.F64>();
        //    scoreH = new Dictionary<Face, FixMath.F64>();
        //    predecessor = new Dictionary<Face, Face>();
        
        //    Intersection loc;
        //    Edge locEdge;
        //    Vertex locVertex;
        //    FixMath.F64 distance;
        //    FixMath.F64Vec2 p1;
        //    FixMath.F64Vec2 p2;
        //    FixMath.F64Vec2 p3;
        //    //
        //    loc = Geom2D.locatePosition(fromX, fromY, _mesh);
        //    {
        //        if (loc is Intersection_EVertex v0)
        //        {
        //            locVertex = v0.vertex;
        //            return;
        //        }
        //        else if (loc is Intersection_EEdge e0)
        //        {
        //            locEdge = e0.edge;
        //            return;
        //        }
        //        else if (loc is Intersection_EFace f0)
        //        {
        //            fromFace = f0.face;
        //        }
        //    }
            

        //    loc = Geom2D.locatePosition(toX, toY, _mesh);
        //    {
        //        if (loc is Intersection_EVertex v1)
        //        {
        //            locVertex = v1.vertex;
        //            toFace = locVertex._edge.get_leftFace();
        //        }
        //        else if (loc is Intersection_EEdge e1)
        //        {
        //            locEdge = e1.edge;
        //            toFace = locEdge.get_leftFace();
        //        }
        //        else if (loc is Intersection_EFace f1)
        //        {
        //            toFace = f1.face;
        //        }
        //    }

        //    /*
        //    fromFace.colorDebug = 0xFF0000;
        //    toFace.colorDebug = 0xFF0000;
        //    Debug.trace( "from face: " + fromFace );
        //    Debug.trace( "to face: " + toFace );
        //    */

        //    var iterTimes = 0;
        //    sortedOpenedFaces.Add(fromFace);
        //    entryEdges[fromFace] = null;
        //    entryX[fromFace] = fromX;
        //    entryY[fromFace] = fromY;
        //    scoreG[fromFace] = FixMath.F64.Zero;
        //    var dist = FixMath.F64.Sqrt((toX - fromX) * (toX - fromX) + (toY - fromY) * (toY - fromY));
        //    scoreH[fromFace] = dist;
        //    scoreF[fromFace] = dist;

        //    Edge innerEdge;
        //    Face neighbourFace;
        //    var f = FixMath.F64.Zero;
        //    var g = FixMath.F64.Zero;
        //    var h = FixMath.F64.Zero;
        //    var fromPoint = FixMath.F64Vec2.Zero;
        //    var entryPoint = FixMath.F64Vec2.Zero;
        //    var distancePoint = FixMath.F64Vec2.Zero;
        //    var fillDatas = false;
        //    while (true)
        //    {
        //        ++iterTimes;
        //        // no path found
        //        if (sortedOpenedFaces.Count == 0 || iterTimes >= iterMaxTimes)
        //        {
        //            //Debug.trace("AStar no path found");
        //            //curFace = null;
        //            break;
        //        }  // we reached the target face  

        //        curFace = sortedOpenedFaces[0];
        //        sortedOpenedFaces.RemoveAt(0);
        //        if (curFace == toFace)
        //            break;
        //        // we continue the search  
        //        iterEdge.set_fromFace(curFace);
        //        while ((innerEdge = iterEdge.next()) != null)
        //        {
        //            if (innerEdge._isConstrained)
        //                continue;
        //            neighbourFace = innerEdge.get_rightFace();
        //            if (!closedFaces.TryGetValue(neighbourFace, out var closed) || !closed)
        //            {
        //                if (curFace != fromFace && _radius > 0 && !isWalkableByRadius(entryEdges[curFace], curFace, innerEdge))
        //                {
        //                    //                            Debug.trace("- NOT WALKABLE -");
        //                    //                            Debug.trace( "from ", hxDaedalusEdge(__entryEdges[__curFace]).originVertex.id, hxDaedalusEdge(__entryEdges[__curFace]).destinationVertex.id );
        //                    //                            Debug.trace( "to", innerEdge.originVertex.id, innerEdge.destinationVertex.id );
        //                    //                            Debug.trace("----------------");
        //                    continue;
        //                }

        //                fromPoint.X = entryX[curFace];
        //                fromPoint.Y = entryY[curFace];
        //                entryPoint.X = (innerEdge.get_originVertex()._pos.X + innerEdge.get_destinationVertex()._pos.X) / 2;
        //                entryPoint.Y = (innerEdge.get_originVertex()._pos.Y + innerEdge.get_destinationVertex()._pos.Y) / 2;
        //                distancePoint.X = entryPoint.X - toX;
        //                distancePoint.Y = entryPoint.Y - toY;
        //                h = FixMath.F64Vec2.LengthFast(distancePoint);
        //                distancePoint.X = fromPoint.X - entryPoint.X;
        //                distancePoint.Y = fromPoint.Y - entryPoint.Y;
        //                g = scoreG[curFace] + FixMath.F64Vec2.LengthFast(distancePoint);
        //                f = h + g;
        //                fillDatas = false;
        //                if (!openedFaces.TryGetValue(neighbourFace, out var open) || !open)
        //                {
        //                    sortedOpenedFaces.Add(neighbourFace);
        //                    openedFaces[neighbourFace] = true;
        //                    fillDatas = true;
        //                }
        //                else if (scoreF[neighbourFace] > f)
        //                {
        //                    fillDatas = true;
        //                }
        //                if (fillDatas)
        //                {
        //                    entryEdges[neighbourFace] = innerEdge;
        //                    entryX[neighbourFace] = entryPoint.X;
        //                    entryY[neighbourFace] = entryPoint.Y;
        //                    scoreF[neighbourFace] = f;
        //                    scoreG[neighbourFace] = g;
        //                    scoreH[neighbourFace] = h;
        //                    predecessor[neighbourFace] = curFace;
        //                }
        //            }
        //        }  //  

        //        openedFaces[curFace] = false;
        //        closedFaces[curFace] = true;
        //        sortedOpenedFaces.Sort(sortingFaces);
        //    }  // if we didn't find a path  

        //    if (curFace == null)
        //        return;  // else we build the path  ;
        //    isPartial = curFace != toFace;

        //    resultListFaces.Add(curFace);
        //    //curFace.colorDebug = 0x0000FF;
        //    while (curFace != fromFace)
        //    {
        //        resultListEdges.Insert(0, entryEdges[curFace]);
        //        //entryEdges[__curFace].colorDebug = 0xFFFF00;
        //        //entryEdges[__curFace].oppositeEdge.colorDebug = 0xFFFF00;
        //        curFace = predecessor[curFace];
        //        //curFace.colorDebug = 0x0000FF;
        //        resultListFaces.Insert(0, curFace);
        //    }
        //}

        //// faces with low distance value are at the end of the array
        //int sortingFaces(Face a, Face b)
        //{
        //    return scoreF[a].CompareTo(scoreG[b]);
        //}

        bool isWalkableByRadius(Edge fromEdge, Face throughFace, Edge toEdge) 
        {
            Vertex vA = null;  // the vertex on fromEdge not on toEdge  
            Vertex vB = null;  // the vertex on toEdge not on fromEdge  
            Vertex vC = null;  // the common vertex of the 2 edges (pivot)  

            // we identify the points
            if (fromEdge.get_originVertex() == toEdge.get_originVertex())
            {
                vA = fromEdge.get_destinationVertex();
                vB = toEdge.get_destinationVertex();
                vC = fromEdge.get_originVertex();
            }
            else if (fromEdge.get_destinationVertex() == toEdge.get_destinationVertex())
            {
                vA = fromEdge.get_originVertex();
                vB = toEdge.get_originVertex();
                vC = fromEdge.get_destinationVertex();
            }
            else if (fromEdge.get_originVertex() == toEdge.get_destinationVertex())
            {
                vA = fromEdge.get_destinationVertex();
                vB = toEdge.get_originVertex();
                vC = fromEdge.get_originVertex();
            }
            else if (fromEdge.get_destinationVertex() == toEdge.get_originVertex())
            {
                vA = fromEdge.get_originVertex();
                vB = toEdge.get_destinationVertex();
                vC = fromEdge.get_destinationVertex();
            }

            FixMath.F64 dot;
            bool result;
            FixMath.F64 distSquared;

            // if we have a right or obtuse angle on CAB
            dot = (vC._pos.X - vA._pos.X) * (vB._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vB._pos.Y - vA._pos.Y);
            if (dot <= 0)
            {
                // we compare length of AC with radius
                distSquared = (vC._pos.X - vA._pos.X) * (vC._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vC._pos.Y - vA._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }  // if we have a right or obtuse angle on CBA  



            dot = (vC._pos.X - vB._pos.X) * (vA._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vA._pos.Y - vB._pos.Y);
            if (dot <= 0)
            {
                // we compare length of BC with radius
                distSquared = (vC._pos.X - vB._pos.X) * (vC._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vC._pos.Y - vB._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }  // we identify the adjacent edge (facing pivot vertex)  



            Edge adjEdge;
            if (throughFace._edge != fromEdge && throughFace._edge._oppositeEdge != fromEdge && throughFace._edge != toEdge && throughFace._edge._oppositeEdge != toEdge)
            {
                adjEdge = throughFace._edge;
            }
            else if (throughFace._edge.get_nextLeftEdge() != fromEdge && throughFace._edge.get_nextLeftEdge()._oppositeEdge != fromEdge && throughFace._edge.get_nextLeftEdge() != toEdge && throughFace._edge.get_nextLeftEdge()._oppositeEdge != toEdge)
            {
                adjEdge = throughFace._edge.get_nextLeftEdge();
            }
            else
            {
                adjEdge = throughFace._edge.get_prevLeftEdge();
            }
            // if the adjacent edge is constrained, we check the distance of orthognaly projected
            if (adjEdge._isConstrained)
            {
                var proj = new FixMath.F64Vec2(vC._pos.X, vC._pos.Y);
                Geom2D.projectOrthogonaly(ref proj, adjEdge);
                distSquared = (proj.X - vC._pos.X) * (proj.X - vC._pos.X) + (proj.Y - vC._pos.Y) * (proj.Y - vC._pos.Y);
                if (distSquared >= diameterSquared)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            // if the adjacent is not constrained
            else
            {
                var distSquaredA = (vC._pos.X - vA._pos.X) * (vC._pos.X - vA._pos.X) + (vC._pos.Y - vA._pos.Y) * (vC._pos.Y - vA._pos.Y);
                var distSquaredB = (vC._pos.X - vB._pos.X) * (vC._pos.X - vB._pos.X) + (vC._pos.Y - vB._pos.Y) * (vC._pos.Y - vB._pos.Y);
                if (distSquaredA < diameterSquared || distSquaredB < diameterSquared)
                {
                    return false;
                }
                else
                {
                    var vFaceToCheck = new List<Face>();
                    var vFaceIsFromEdge = new List<Edge>();
                    var facesDone = new Dictionary<Face, bool>();
                    vFaceIsFromEdge.Add(adjEdge);
                    if (adjEdge.get_leftFace() == throughFace)
                    {
                        vFaceToCheck.Add(adjEdge.get_rightFace());
                        facesDone[adjEdge.get_rightFace()] = true;
                    }
                    else
                    {
                        vFaceToCheck.Add(adjEdge.get_leftFace());
                        facesDone[adjEdge.get_leftFace()] = true;
                    }

                    Face currFace;
                    Edge faceFromEdge;
                    Edge currEdgeA;
                    Face nextFaceA;
                    Edge currEdgeB;
                    Face nextFaceB;
                    while (vFaceToCheck.Count > 0)
                    {
                        currFace = vFaceToCheck[0];
                        vFaceToCheck.RemoveAt(0);

                        faceFromEdge = vFaceIsFromEdge[0];
                        vFaceIsFromEdge.RemoveAt(0);

                        if (currFace._edge == faceFromEdge || currFace._edge == faceFromEdge._oppositeEdge)
                        {
                            // we identify the 2 edges to evaluate
                            currEdgeA = currFace._edge.get_nextLeftEdge();
                            currEdgeB = currFace._edge.get_nextLeftEdge().get_nextLeftEdge();
                        }
                        else if (currFace._edge.get_nextLeftEdge() == faceFromEdge || currFace._edge.get_nextLeftEdge() == faceFromEdge._oppositeEdge)
                        {
                            // we identify the faces related to the 2 edges
                            currEdgeA = currFace._edge;
                            currEdgeB = currFace._edge.get_nextLeftEdge().get_nextLeftEdge();
                        }
                        else
                        {
                            currEdgeA = currFace._edge;
                            currEdgeB = currFace._edge.get_nextLeftEdge();
                        }

                        if (currEdgeA.get_leftFace() == currFace)
                        {
                            nextFaceA = currEdgeA.get_rightFace();
                        }
                        else
                        {
                            nextFaceA = currEdgeA.get_leftFace();
                        }
                        if (currEdgeB.get_leftFace() == currFace)
                        {
                            nextFaceB = currEdgeB.get_rightFace();
                        }
                        else
                        {
                            nextFaceB = currEdgeB.get_leftFace();
                        }
                        // we check if the next face is not already in pipe
                        // and if the edge A is close to pivot vertex
                        var nextFaceA_not_done = !facesDone.TryGetValue(nextFaceA, out var doneA) || !doneA;
                        if (nextFaceA_not_done && Geom2D.distanceSquaredVertexToEdge(vC, currEdgeA) < diameterSquared)
                        {
                            // if the edge is constrained
                            if (currEdgeA._isConstrained)
                            {
                                // so it is not walkable
                                return false;
                            }
                            else
                            {
                                // if the edge is not constrained, we continue the search
                                vFaceToCheck.Add(nextFaceA);
                                vFaceIsFromEdge.Add(currEdgeA);
                                facesDone[nextFaceA] = true;
                            }
                        }  // and if the edge B is close to pivot vertex    // we check if the next face is not already in pipe  

                        var nextFaceB_not_done = !facesDone.TryGetValue(nextFaceB, out var doneB) || !doneB;
                        if (nextFaceB_not_done && Geom2D.distanceSquaredVertexToEdge(vC, currEdgeB) < diameterSquared)
                        {
                            // if the edge is constrained
                            if (currEdgeB._isConstrained)
                            {
                                // so it is not walkable
                                return false;
                            }
                            else
                            {
                                // if the edge is not constrained, we continue the search
                                vFaceToCheck.Add(nextFaceB);
                                vFaceIsFromEdge.Add(currEdgeB);
                                facesDone[nextFaceB] = true;
                            }
                        }
                    }  // if we didn't previously meet a constrained edge  

                    return true;
                }
            }

            return true;
        }
    }
}
