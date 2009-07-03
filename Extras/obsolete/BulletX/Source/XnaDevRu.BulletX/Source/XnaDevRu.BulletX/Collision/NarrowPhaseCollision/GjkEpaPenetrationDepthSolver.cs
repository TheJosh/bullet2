/*
  Bullet for XNA Copyright (c) 2003-2007 Vsevolod Klementjev http://www.codeplex.com/xnadevru
  Bullet original C++ version Copyright (c) 2003-2007 Erwin Coumans http://bulletphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

using System;
using System.Collections.Generic;
using System.Text;
using Microsoft.Xna.Framework;

namespace XnaDevRu.BulletX
{
	/// <summary>
	/// EpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to
	/// calculate the penetration depth between two convex shapes.
	/// </summary>
	public class GjkEpaPenetrationDepthSolver : IConvexPenetrationDepthSolver
	{
		public bool CalculatePenetrationDepth(ISimplexSolver simplexSolver, ConvexShape convexA, ConvexShape convexB, Matrix transformA, Matrix transformB, Vector3 vector, out Vector3 ptrA, out Vector3 ptrB, IDebugDraw debugDraw)
		{
			float radialmargin = 0;

			GjkEpaSolver.Results results;
			if (GjkEpaSolver.Collide(convexA, transformA,
									 convexB, transformB,
									 radialmargin, out results))
			{
				//	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
				//resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
				ptrA = results.Witnesses[0];
				ptrB = results.Witnesses[1];
				return true;
			}
			ptrA = new Vector3();
			ptrB = new Vector3();

			return false;
		}
	}
}
