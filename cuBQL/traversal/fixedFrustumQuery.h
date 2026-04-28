// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "cuBQL/traversal/fixedAnyShapeQuery.h"

namespace cuBQL {

  /*! A view frustum represented as 6 half-planes.  For each plane
      (nx, ny, nz, d), a point p is on the "inside" iff
      nx*p.x + ny*p.y + nz*p.z + d >= 0.

      Plane order (by convention, not enforced):
        0 = left, 1 = right, 2 = bottom, 3 = top, 4 = near, 5 = far

      Planes do NOT need to be normalised; the p-vertex test only
      cares about the sign of the dot product. */
  struct Frustum {
    vec_t<float,4> planes[6];
  };

  /*! Conservative AABB-vs-frustum overlap test (p-vertex method).
      Returns false only when the box is entirely outside at least one
      frustum plane --- never produces false negatives. */
  inline __cubql_both
  bool frustumIntersectsBox(const Frustum &frustum,
                            const box_t<float,3> &box)
  {
    for (int i = 0; i < 6; i++) {
      const vec_t<float,4> &pl = frustum.planes[i];
      // p-vertex: the AABB corner most in the direction of the normal
      float px = (pl.x >= 0.f) ? box.upper.x : box.lower.x;
      float py = (pl.y >= 0.f) ? box.upper.y : box.lower.y;
      float pz = (pl.z >= 0.f) ? box.upper.z : box.lower.z;
      if (pl.x * px + pl.y * py + pl.z * pz + pl.w < 0.f)
        return false;
    }
    return true;
  }

  namespace fixedFrustumQuery {

    // ================================================================
    // INTERFACE
    // ================================================================

    /*! Traverse the BVH and call `lambda(int primID)->int` for every
        primitive whose BVH leaf overlaps the frustum.  The lambda
        returns CUBQL_TERMINATE_TRAVERSAL or CUBQL_CONTINUE_TRAVERSAL. */
    template<typename T, int D, typename Lambda>
    inline __cubql_both
    void forEachPrim(const Lambda &lambdaToCallOnEachPrim,
                     const BinaryBVH<T,D> bvh,
                     const Frustum &frustum,
                     bool dbg=false);

    /*! Same, but the lambda receives the whole leaf:
        lambda(const uint32_t *primIDs, uint32_t count)->int */
    template<typename T, int D, typename Lambda>
    inline __cubql_both
    void forEachLeaf(const Lambda &lambdaToCallOnEachLeaf,
                     const BinaryBVH<T,D> bvh,
                     const Frustum &frustum,
                     bool dbg=false);

    /*! WideBVH overloads */
    template<typename T, int D, int W, typename Lambda>
    inline __cubql_both
    void forEachPrim(const Lambda &lambdaToCallOnEachPrim,
                     const WideBVH<T,D,W> bvh,
                     const Frustum &frustum,
                     bool dbg=false);

    template<typename T, int D, int W, typename Lambda>
    inline __cubql_both
    void forEachLeaf(const Lambda &lambdaToCallOnEachLeaf,
                     const WideBVH<T,D,W> bvh,
                     const Frustum &frustum,
                     bool dbg=false);


    // ================================================================
    // IMPLEMENTATION  ---  BinaryBVH
    // ================================================================

    template<typename T, int D, typename Lambda>
    inline __cubql_both
    void forEachLeaf(const Lambda &lambdaToCallOnEachLeaf,
                     const BinaryBVH<T,D> bvh,
                     const Frustum &frustum,
                     bool dbg)
    {
      using node_t = typename BinaryBVH<T,D>::Node;
      typename node_t::Admin traversalStack[64], *stackPtr = traversalStack;
      typename node_t::Admin node = bvh.nodes[0].admin;

      while (true) {
        // ---- descend through inner nodes ----
        while (true) {
          if (node.count != 0)
            break;   // reached a leaf

          uint32_t n0Idx = (uint32_t)node.offset + 0;
          uint32_t n1Idx = (uint32_t)node.offset + 1;
          node_t n0 = bvh.nodes[n0Idx];
          node_t n1 = bvh.nodes[n1Idx];
          bool o0 = frustumIntersectsBox(frustum, n0.bounds);
          bool o1 = frustumIntersectsBox(frustum, n1.bounds);

          if (o0) {
            if (o1) *stackPtr++ = n1.admin;
            node = n0.admin;
          } else {
            if (o1) {
              node = n1.admin;
            } else {
              node.count = 0;
              break;   // dead end
            }
          }
        }

        // ---- process leaf ----
        if (node.count != 0) {
          int leafResult =
            lambdaToCallOnEachLeaf(bvh.primIDs + node.offset,
                                   (uint32_t)node.count);
          if (leafResult == CUBQL_TERMINATE_TRAVERSAL)
            return;
        }

        // ---- pop ----
        if (stackPtr == traversalStack)
          return;
        node = *--stackPtr;
      }
    }

    template<typename T, int D, typename Lambda>
    inline __cubql_both
    void forEachPrim(const Lambda &lambdaToCallOnEachPrim,
                     const BinaryBVH<T,D> bvh,
                     const Frustum &frustum,
                     bool dbg)
    {
      auto leafCode = [&lambdaToCallOnEachPrim]
        (const uint32_t *primIDs, size_t numPrims) -> int
      {
        for (int i = 0; i < (int)numPrims; i++)
          if (lambdaToCallOnEachPrim(primIDs[i]) == CUBQL_TERMINATE_TRAVERSAL)
            return CUBQL_TERMINATE_TRAVERSAL;
        return CUBQL_CONTINUE_TRAVERSAL;
      };
      forEachLeaf(leafCode, bvh, frustum, dbg);
    }


    // ================================================================
    // IMPLEMENTATION  ---  WideBVH
    // ================================================================

    template<typename T, int D, int W, typename Lambda>
    inline __cubql_both
    void forEachLeaf(const Lambda &lambdaToCallOnEachLeaf,
                     const WideBVH<T,D,W> bvh,
                     const Frustum &frustum,
                     bool dbg)
    {
      struct StackEntry {
        uint64_t nodeID  : 48;
        uint64_t childID : 16;
      };
      StackEntry traversalStack[64], *stackPtr = traversalStack;
      StackEntry current;
      current.nodeID  = 0;
      current.childID = 0;

      typename WideBVH<T,D,W>::Node::Child child;
      while (true) {
        while (true) {
          child = bvh.nodes[current.nodeID].children[current.childID];
          if (!child.valid
              || !frustumIntersectsBox(frustum, child.bounds)) {
            if (current.childID + 1 < W)
              current.childID = current.childID + 1;
            else if (stackPtr > traversalStack)
              current = *--stackPtr;
            else
              return;
          } else if (child.count == 0) {
            // inner node -- push sibling, descend
            if (current.childID + 1 < W) {
              current.childID++;
              *stackPtr++ = current;
            }
            current.nodeID  = child.offset;
            current.childID = 0;
          } else {
            break;   // leaf
          }
        }

        // process leaf
        int leafResult =
          lambdaToCallOnEachLeaf(bvh.primIDs + child.offset,
                                 (uint32_t)child.count);
        if (leafResult == CUBQL_TERMINATE_TRAVERSAL)
          return;

        if (current.childID + 1 < W)
          current.childID = current.childID + 1;
        else if (stackPtr > traversalStack)
          current = *--stackPtr;
        else
          return;
      }
    }

    template<typename T, int D, int W, typename Lambda>
    inline __cubql_both
    void forEachPrim(const Lambda &lambdaToCallOnEachPrim,
                     const WideBVH<T,D,W> bvh,
                     const Frustum &frustum,
                     bool dbg)
    {
      auto leafCode = [&lambdaToCallOnEachPrim]
        (const uint32_t *primIDs, size_t numPrims) -> int
      {
        for (int i = 0; i < (int)numPrims; i++)
          if (lambdaToCallOnEachPrim(primIDs[i]) == CUBQL_TERMINATE_TRAVERSAL)
            return CUBQL_TERMINATE_TRAVERSAL;
        return CUBQL_CONTINUE_TRAVERSAL;
      };
      forEachLeaf(leafCode, bvh, frustum, dbg);
    }

  } // ::cuBQL::fixedFrustumQuery
} // ::cuBQL
