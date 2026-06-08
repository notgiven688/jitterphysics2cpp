#pragma once

#include <cmath>
#include <stdexcept>

#include <Jitter2/Collision/Shapes/Shape.hpp>
#include <Jitter2/DebugCheck.hpp>
#include <Jitter2/LinearMath/MathHelper.hpp>

namespace Jitter2::Collision::Shapes
{

// Represents a shape wrapper defined by an original shape and an affine transformation (translation and linear map).
class TransformedShape final : public RigidBodyShape
{
public:
    TransformedShape(
        RigidBodyShape& shape,
        const LinearMath::JVector& translation,
        const LinearMath::JMatrix& transformation)
        : originalShape_(&shape),
          translation_(ArgumentCheck::Finite(translation, "translation")),
          transformation_(ArgumentCheck::Finite(transformation, "transform"))
    {
        AnalyzeTransformation();
        UpdateWorldBoundingBox();
    }

    // Constructs a transformed shape with a translation (offset), assuming identity rotation/scale.

    TransformedShape(RigidBodyShape& shape, const LinearMath::JVector& translation)
        : TransformedShape(shape, translation, LinearMath::JMatrix::Identity())
    {
    }

    // Constructs a transformed shape with a linear transformation (rotation, scale, or shear),
    // assuming zero translation.

    TransformedShape(RigidBodyShape& shape, const LinearMath::JMatrix& transformation)
        : TransformedShape(shape, LinearMath::JVector::Zero(), transformation)
    {
    }

    [[nodiscard]] RigidBodyShape& OriginalShape() const { return *originalShape_; }
    [[nodiscard]] LinearMath::JVector Translation() const { return translation_; }
    [[nodiscard]] LinearMath::JMatrix Transformation() const { return transformation_; }

    void Translation(const LinearMath::JVector& value)
    {
        DebugCheck::IsFinite(value, "value");
        translation_ = value;
        UpdateWorldBoundingBox();
    }

    void Transformation(const LinearMath::JMatrix& value)
    {
        DebugCheck::IsFinite(value, "value");
        transformation_ = value;
        AnalyzeTransformation();
        UpdateWorldBoundingBox();
    }


    void SupportMap(const LinearMath::JVector& direction, LinearMath::JVector& result) const override
    {
        if (type_ == TransformationType::Identity)
        {
            originalShape_->SupportMap(direction, result);
            result += translation_;
            return;
        }

        const LinearMath::JVector transformedDirection =
            LinearMath::JMatrix::TransposedTransform(direction, transformation_);
        originalShape_->SupportMap(transformedDirection, result);
        result = transformation_ * result + translation_;
    }


    void GetCenter(LinearMath::JVector& point) const override
    {
        originalShape_->GetCenter(point);
        point = transformation_ * point + translation_;
    }

    void CalculateBoundingBox(
        const LinearMath::JQuaternion& orientation,
        const LinearMath::JVector& position,
        LinearMath::JBoundingBox& box) const override
    {
        if (type_ != TransformationType::General)
        {
            const LinearMath::JQuaternion rotation = LinearMath::JQuaternion::CreateFromMatrix(transformation_);
            originalShape_->CalculateBoundingBox(
                orientation * rotation,
                LinearMath::JQuaternion::Transform(translation_, orientation) + position,
                box);
            return;
        }

        const LinearMath::JMatrix orientationTranspose =
            LinearMath::JMatrix::Transpose(LinearMath::JMatrix::CreateFromQuaternion(orientation));

        LinearMath::JVector result;
        LinearMath::JVector direction = orientationTranspose.GetColumn(0);
        SupportMap(direction, result);
        box.Max.X = LinearMath::JVector::Dot(direction, result);
        SupportMap(-direction, result);
        box.Min.X = LinearMath::JVector::Dot(direction, result);

        direction = orientationTranspose.GetColumn(1);
        SupportMap(direction, result);
        box.Max.Y = LinearMath::JVector::Dot(direction, result);
        SupportMap(-direction, result);
        box.Min.Y = LinearMath::JVector::Dot(direction, result);

        direction = orientationTranspose.GetColumn(2);
        SupportMap(direction, result);
        box.Max.Z = LinearMath::JVector::Dot(direction, result);
        SupportMap(-direction, result);
        box.Min.Z = LinearMath::JVector::Dot(direction, result);

        box.Min += position;
        box.Max += position;
    }

    void CalculateMassInertia(
        LinearMath::JMatrix& inertia,
        LinearMath::JVector& centerOfMass,
        Real& mass) const override
    {
        LinearMath::JMatrix originalInertia;
        LinearMath::JVector originalCenter;

        originalShape_->CalculateMassInertia(originalInertia, originalCenter, mass);

        centerOfMass = transformation_ * originalCenter + translation_;
        const Real determinant = std::abs(transformation_.Determinant());
        mass *= determinant;

        const Real halfTrace = originalInertia.Trace() * static_cast<Real>(0.5);
        const LinearMath::JMatrix secondMoment = halfTrace * LinearMath::JMatrix::Identity() - originalInertia;
        const LinearMath::JMatrix transformedSecondMoment =
            determinant * transformation_ * secondMoment * LinearMath::JMatrix::Transpose(transformation_);

        inertia = transformedSecondMoment.Trace() * LinearMath::JMatrix::Identity() - transformedSecondMoment;
        const LinearMath::JMatrix parallelAxis =
            mass * (LinearMath::JMatrix::Identity() * centerOfMass.LengthSquared()
                - LinearMath::JMatrix::Outer(centerOfMass, centerOfMass));
        inertia = inertia + parallelAxis;
    }

private:
    enum class TransformationType
    {
        Identity,
        Rotation,
        General
    };

    void AnalyzeTransformation()
    {
        if (LinearMath::MathHelper::IsRotationMatrix(transformation_))
        {
            const LinearMath::JMatrix delta = transformation_ - LinearMath::JMatrix::Identity();
            type_ = LinearMath::MathHelper::UnsafeIsZero(delta)
                ? TransformationType::Identity
                : TransformationType::Rotation;
        }
        else
        {
            type_ = TransformationType::General;
        }
    }

    RigidBodyShape* originalShape_;
    LinearMath::JVector translation_ = LinearMath::JVector::Zero();
    LinearMath::JMatrix transformation_ = LinearMath::JMatrix::Identity();
    TransformationType type_ = TransformationType::Identity;
};

} // namespace Jitter2::Collision::Shapes
