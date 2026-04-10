//
// SPDX-License-Identifier: Apache-2.0
//
// PicoGK ("peacock") is a compact software kernel for computational geometry,
// specifically for use in Computational Engineering Models (CEM).
//
// For more information, please visit https://picogk.org
//
// PicoGK is developed and maintained by LEAP 71 - © 2023-2026 by LEAP 71
// https://leap71.com
//
// Computational Engineering will profoundly change our physical world in the
// years ahead. Thank you for being part of the journey.
//
// We have developed this library to be used widely, for both commercial and
// non-commercial projects alike. Therefore, we have released it under a
// permissive open-source license.
//
// The foundation of PicoGK is a thin layer on top of the powerful open-source
// OpenVDB project, which in turn uses many other Free and Open Source Software
// libraries. We are grateful to be able to stand on the shoulders of giants.
//
// LEAP 71 licenses this file to you under the Apache License, Version 2.0
// (the "License"); you may not use this file except in compliance with the
// License. You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, THE SOFTWARE IS
// PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
//
// See the License for the specific language governing permissions and
// limitations under the License.
//

using System.Numerics;

namespace PicoGK
{
    /// <summary>
    /// Generator for a 4×110 speed skate frame using PicoGK SDF techniques.
    ///
    /// Architecture:
    ///   Wheel diameter      : 110 mm
    ///   Wheel spacing       : 112 mm center-to-center (2 mm clearance)
    ///   Mounting standard   : 195 mm longitudinal
    ///   Axle diameter       : 8 mm
    ///   Total frame length  : 336 mm  (Axle 1 → Axle 4)
    ///
    /// Coordinate system:
    ///   X  along the frame length (front = 0, rear = 336 mm)
    ///   Y  lateral  (frame centreline = 0)
    ///   Z  vertical (axle plane = 0, boot-sole plane = +fFrameHeightMM)
    /// </summary>
    public class SpeedSkateFrame
    {
        // ── Frame architecture constants ────────────────────────────────────────

        /// <summary>Wheel diameter in mm.</summary>
        public const float fWheelDiameterMM     = 110f;

        /// <summary>Center-to-center wheel spacing in mm (2 mm clearance between wheels).</summary>
        public const float fWheelSpacingMM      = 112f;

        /// <summary>Longitudinal distance between the two mounting bolt positions in mm.</summary>
        public const float fMountingSpanMM      = 195f;

        /// <summary>Axle hole diameter in mm.</summary>
        public const float fAxleDiameterMM      = 8f;

        /// <summary>Total frame length from Axle 1 to Axle 4 in mm.</summary>
        public const float fFrameLengthMM       = 336f;

        /// <summary>Frame height from axle centreline to boot-sole mounting surface in mm.</summary>
        public const float fFrameHeightMM       = 28f;

        /// <summary>Outer frame lateral width (wall-to-wall) in mm.</summary>
        public const float fFrameWidthMM        = 14f;

        /// <summary>Nominal outer shell wall thickness used when hollowing the body in mm.</summary>
        public const float fWallThicknessMM     = 3.0f;

        /// <summary>Target gyroid cell size in mm.</summary>
        public const float fGyroidCellSizeMM    = 5f;

        // ── Key structural point definitions ────────────────────────────────────
        //
        // Axle centres sit at Z = 0 (wheel-axle plane) on the frame centreline.
        // Mount centres sit at Z = fFrameHeightMM on the frame centreline.
        // The mounting bolt centres are symmetrically placed around the frame midpoint.

        /// <summary>Front axle centre position.</summary>
        public static readonly Vector3 vecAxle1 = new(0f,                         0f, 0f);

        /// <summary>Second axle centre position.</summary>
        public static readonly Vector3 vecAxle2 = new(fWheelSpacingMM,            0f, 0f);

        /// <summary>Third axle centre position.</summary>
        public static readonly Vector3 vecAxle3 = new(2f * fWheelSpacingMM,       0f, 0f);

        /// <summary>Rear axle centre position.</summary>
        public static readonly Vector3 vecAxle4 = new(fFrameLengthMM,             0f, 0f);

        // Mounting-bolt centres (195 mm span, centred on the 336 mm frame)
        // X offset from front of frame to the front mounting bolt: (336 - 195) / 2 = 70.5 mm
        static readonly float fMountX0 = (fFrameLengthMM - fMountingSpanMM) * 0.5f;

        /// <summary>Front mounting bolt centre position.</summary>
        public static readonly Vector3 vecMount1 = new(fMountX0,                    0f, fFrameHeightMM);

        /// <summary>Rear mounting bolt centre position.</summary>
        public static readonly Vector3 vecMount2 = new(fMountX0 + fMountingSpanMM,  0f, fFrameHeightMM);

        // ── Public API ──────────────────────────────────────────────────────────

        /// <summary>
        /// Generates the main frame body by connecting the four axle points to
        /// the two mounting points using smoothed box-SDFs (signed distance fields).
        ///
        /// The body is assembled from three smooth-union primitives:
        ///   • Front section  – rounded box spanning Axle 1 → Mount 1
        ///   • Rear  section  – rounded box spanning Mount 2 → Axle 4
        ///   • Arch  bridge   – thicker, taller rounded box spanning Mount 1 → Mount 2,
        ///                      providing the reinforcement needed for the 195 mm
        ///                      mounting stress (front-two / rear-two wheel bridge)
        ///
        /// A final triple-offset (Smoothen) pass removes voxelisation artefacts.
        /// </summary>
        /// <returns>Frame body as a voxel field.</returns>
        public static Voxels voxGenerateFrameBody()
        {
            // Generous bounding box: the arch raises the body by fArchExtraHeightMM
            // above fFrameHeightMM, so add extra headroom in Z.
            BBox3 oBounds = new(
                new Vector3(-8f,  -(fFrameWidthMM * 0.5f + 8f), -8f),
                new Vector3(fFrameLengthMM + 8f,
                             fFrameWidthMM * 0.5f + 8f,
                             fFrameHeightMM + 15f));

            Voxels vox = new(new SmoothedFrameBody(), oBounds);
            vox.Smoothen(1.0f);
            return vox;
        }

        /// <summary>
        /// Generates a variable-density gyroid lattice clipped to the frame body.
        ///
        /// The gyroid cell size is 5 mm.  A proximity-based scalar field drives
        /// the iso-level threshold so that wall thickness is highest (0.35) near
        /// the axle and mounting points and lowest (0.10) in the centre of each
        /// span, producing a weight-optimised internal structure.
        /// </summary>
        /// <param name="voxFrameBody">Frame body voxel field used as the clipping mask.</param>
        /// <returns>Gyroid lattice voxel field clipped to the frame body.</returns>
        public static Voxels voxGenerateGyroidLattice(in Voxels voxFrameBody)
        {
            Voxels voxLattice = new(voxFrameBody);
            voxLattice.IntersectImplicit(new VariableDensityGyroid(
                vecAxle1, vecAxle2, vecAxle3, vecAxle4,
                vecMount1, vecMount2,
                fGyroidCellSizeMM));
            return voxLattice;
        }

        /// <summary>
        /// Generates external diagonal reinforcement ribs that follow the
        /// lines of force during a skating push (~45–60 °, midpoint 52.5 °).
        ///
        /// For each mounting point two ribs are created:
        ///   • a front-leaning rib (toward the front of the frame)
        ///   • a rear-leaning  rib (toward the rear  of the frame)
        ///
        /// Both ribs run from mount height (Z = fFrameHeightMM) down to the
        /// axle plane (Z = 0) at the calculated horizontal offset.
        ///
        /// The rib geometry is produced with a <see cref="Lattice"/> and then
        /// trimmed to the frame body boundary using
        /// <see cref="Voxels.BoolIntersect"/>.
        /// </summary>
        /// <param name="voxFrameBody">Frame body used to mask the rib geometry.</param>
        /// <returns>Reinforcement rib voxel field.</returns>
        public static Voxels voxGenerateReinforcementRibs(in Voxels voxFrameBody)
        {
            const float fRibRadiusTopMM    = 3.5f;   // radius at mount end
            const float fRibRadiusBottomMM = 2.0f;   // radius at axle-plane end (tapered)

            // Midpoint of 45–60 ° range
            const float fAngleDeg = 52.5f;
            float fAngleRad       = fAngleDeg * MathF.PI / 180f;

            // Horizontal distance from mount X to rib bottom X at the chosen angle
            float fDx = fFrameHeightMM / MathF.Tan(fAngleRad);

            Lattice lat = new();

            foreach (Vector3 vecMount in new[] { vecMount1, vecMount2 })
            {
                // Front-leaning rib
                Vector3 vecBottomFront = new(vecMount.X - fDx, 0f, 0f);
                lat.AddBeam(vecMount, fRibRadiusTopMM, vecBottomFront, fRibRadiusBottomMM);

                // Rear-leaning rib
                Vector3 vecBottomRear = new(vecMount.X + fDx, 0f, 0f);
                lat.AddBeam(vecMount, fRibRadiusTopMM, vecBottomRear, fRibRadiusBottomMM);
            }

            Voxels voxRibs = new(lat);

            // Clip the ribs to the frame body boundary
            voxRibs.BoolIntersect(voxFrameBody);
            return voxRibs;
        }

        /// <summary>
        /// Builds the complete frame:
        ///   outer shell  +  variable-density gyroid infill  +  diagonal reinforcement ribs.
        /// </summary>
        /// <returns>Complete frame as a voxel field.</returns>
        public static Voxels voxBuildCompleteFrame()
        {
            Voxels voxBody   = voxGenerateFrameBody();
            Voxels voxShell  = voxBody.voxShell(-fWallThicknessMM);
            Voxels voxInfill = voxGenerateGyroidLattice(voxBody);
            Voxels voxRibs   = voxGenerateReinforcementRibs(voxBody);

            voxShell.BoolAdd(voxInfill);
            voxShell.BoolAdd(voxRibs);
            return voxShell;
        }

        // ── Private SDF helper functions ────────────────────────────────────────

        /// <summary>
        /// Signed distance to a rounded box centred at <paramref name="vecCenter"/>
        /// with half-extents <paramref name="vecHalf"/> and corner radius
        /// <paramref name="fRound"/>.
        ///
        /// Based on Inigo Quilez's exact rounded-box SDF.
        /// </summary>
        static float fRoundedBox(   in Vector3 vec,
                                    in Vector3 vecCenter,
                                    in Vector3 vecHalf,
                                    float      fRound)
        {
            Vector3 q = new(
                MathF.Abs(vec.X - vecCenter.X) - vecHalf.X + fRound,
                MathF.Abs(vec.Y - vecCenter.Y) - vecHalf.Y + fRound,
                MathF.Abs(vec.Z - vecCenter.Z) - vecHalf.Z + fRound);

            return   MathF.Min(MathF.Max(q.X, MathF.Max(q.Y, q.Z)), 0f)
                   + new Vector3(MathF.Max(q.X, 0f),
                                 MathF.Max(q.Y, 0f),
                                 MathF.Max(q.Z, 0f)).Length()
                   - fRound;
        }

        /// <summary>
        /// Smooth union of two SDF values using the polynomial C1 blend
        /// by Inigo Quilez, with blending radius <paramref name="k"/>.
        /// </summary>
        static float fSmoothUnion(float fA, float fB, float k)
        {
            float h = MathF.Max(k - MathF.Abs(fA - fB), 0f) / k;
            return MathF.Min(fA, fB) - h * h * k * 0.25f;
        }

        // ── Inner implicit classes ───────────────────────────────────────────────

        /// <summary>
        /// Implicit SDF that represents the smoothed frame body.
        ///
        /// Three rounded-box primitives are merged with smooth-union blending:
        ///   • Front section : spans Axle 1 (X = 0)     → Mount 1 (X = 70.5 mm)
        ///   • Rear  section : spans Mount 2 (X = 265.5) → Axle 4  (X = 336 mm)
        ///   • Arch  bridge  : spans Mount 1 → Mount 2 with extra width (+2 mm)
        ///                     and extra height (+4 mm) to create the reinforced
        ///                     arch that handles the 195 mm mounting stress
        /// </summary>
        sealed class SmoothedFrameBody : IImplicit
        {
            // Geometry for the three primitives (computed once in constructor)
            readonly Vector3 m_vecFrontCenter;
            readonly Vector3 m_vecFrontHalf;
            readonly Vector3 m_vecRearCenter;
            readonly Vector3 m_vecRearHalf;
            readonly Vector3 m_vecArchCenter;
            readonly Vector3 m_vecArchHalf;

            // Shared half-dimensions for rail sections
            const float fHalfW          = fFrameWidthMM  * 0.5f;  // lateral half-width
            const float fHalfH          = fFrameHeightMM * 0.5f;  // vertical half-height

            // Corner radii and blend parameter
            const float fRailRounding   = 2.5f;   // corner radius for front/rear rail
            const float fArchRounding   = 4.0f;   // corner radius for the arch bridge
            const float fBlend          = 12.0f;  // smooth-union blend radius

            // Arch reinforcement margins
            const float fArchExtraWidthMM  = 2.0f;  // extra lateral half-width for arch
            const float fArchExtraHeightMM = 4.0f;  // extra height for arch (upward)

            public SmoothedFrameBody()
            {
                // Front section: from Axle 1 to Mount 1 (full frame height)
                float fFrontLen     = vecMount1.X - vecAxle1.X;        // 70.5 mm
                m_vecFrontCenter    = new(vecAxle1.X + fFrontLen * 0.5f, 0f, fHalfH);
                m_vecFrontHalf      = new(fFrontLen * 0.5f, fHalfW, fHalfH);

                // Rear section: from Mount 2 to Axle 4 (full frame height)
                float fRearLen      = vecAxle4.X - vecMount2.X;        // 70.5 mm
                m_vecRearCenter     = new(vecMount2.X + fRearLen * 0.5f, 0f, fHalfH);
                m_vecRearHalf       = new(fRearLen * 0.5f, fHalfW, fHalfH);

                // Arch bridge: from Mount 1 to Mount 2 with extra reinforcement
                float fArchLen      = vecMount2.X - vecMount1.X;       // 195 mm
                float fArchHalfH    = (fFrameHeightMM + fArchExtraHeightMM) * 0.5f;
                m_vecArchCenter     = new(vecMount1.X + fArchLen * 0.5f, 0f, fArchHalfH);
                m_vecArchHalf       = new(fArchLen * 0.5f,
                                         fHalfW + fArchExtraWidthMM,
                                         fArchHalfH);
            }

            public float fSignedDistance(in Vector3 vec)
            {
                float fFront = fRoundedBox(vec, m_vecFrontCenter, m_vecFrontHalf, fRailRounding);
                float fRear  = fRoundedBox(vec, m_vecRearCenter,  m_vecRearHalf,  fRailRounding);
                float fArch  = fRoundedBox(vec, m_vecArchCenter,  m_vecArchHalf,  fArchRounding);

                float fResult = fSmoothUnion(fFront, fRear, fBlend);
                return fSmoothUnion(fResult, fArch, fBlend);
            }
        }

        /// <summary>
        /// Variable-density gyroid implicit SDF.
        ///
        /// Implements the standard triply-periodic minimal-surface gyroid:
        ///   G(x,y,z) = sin(ωx)·cos(ωy) + sin(ωy)·cos(ωz) + sin(ωz)·cos(ωx)
        /// where ω = 2π / cellSize.
        ///
        /// The iso-level threshold t is modulated by a proximity scalar field:
        ///   t = 0.35  (thick walls) within fInfluenceRadiusMM of any key point
        ///   t = 0.10  (thin  walls) far from all key points
        ///
        /// Calling <see cref="Voxels.IntersectImplicit"/> with this object clips
        /// the frame body to only the gyroid wall material, whose cross-sectional
        /// thickness follows the proximity field.
        /// </summary>
        sealed class VariableDensityGyroid : IImplicit
        {
            readonly Vector3[] m_avecKeyPoints;
            readonly float     m_fOmega;              // angular frequency = 2π / cellSize

            // Proximity-field parameters
            const float fInfluenceRadiusMM = 30f;     // mm – key-point influence radius
            const float fThresholdHigh     = 0.35f;   // iso-level near key points (thick)
            const float fThresholdLow      = 0.10f;   // iso-level far from key points (thin)

            public VariableDensityGyroid(
                Vector3 vecA1, Vector3 vecA2,
                Vector3 vecA3, Vector3 vecA4,
                Vector3 vecM1, Vector3 vecM2,
                float fCellSizeMM)
            {
                m_avecKeyPoints = new[] { vecA1, vecA2, vecA3, vecA4, vecM1, vecM2 };
                m_fOmega        = 2f * MathF.PI / fCellSizeMM;
            }

            public float fSignedDistance(in Vector3 vec)
            {
                float x = vec.X * m_fOmega;
                float y = vec.Y * m_fOmega;
                float z = vec.Z * m_fOmega;

                float fG =   MathF.Sin(x) * MathF.Cos(y)
                           + MathF.Sin(y) * MathF.Cos(z)
                           + MathF.Sin(z) * MathF.Cos(x);

                return MathF.Abs(fG) - fThreshold(vec);
            }

            /// <summary>
            /// Computes the density threshold at <paramref name="vec"/> based on
            /// its proximity to the six key structural points.
            /// The maximum influence of all key points is used (union behaviour).
            /// </summary>
            float fThreshold(in Vector3 vec)
            {
                float fMaxInfl = 0f;

                foreach (Vector3 vecKey in m_avecKeyPoints)
                {
                    float fDist = Vector3.Distance(vec, vecKey);
                    float fInfl = MathF.Max(0f, 1f - fDist / fInfluenceRadiusMM);
                    if (fInfl > fMaxInfl)
                        fMaxInfl = fInfl;
                }

                return fThresholdLow + (fThresholdHigh - fThresholdLow) * fMaxInfl;
            }
        }
    }
}
