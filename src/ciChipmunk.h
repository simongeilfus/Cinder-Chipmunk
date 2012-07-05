#pragma once

#include "cinder/Vector.h"
#include "cinder/Rect.h"
#include "chipmunk.h"
#include <vector>

namespace cp {
    
    inline ci::Vec2f    toCinder( const cpVect &v ){ return ci::Vec2f( v.x, v.y ); }
    inline ci::Rectf    toCinder( const cpBB &r ){ return ci::Rectf( ci::Vec2f( r.l, r.t ), ci::Vec2f( r.r, r.b ) ); }
    
    inline cpVect       toChipmunk( const ci::Vec2f &v ){ cpVect cpv; cpv.x = v.x; cpv.y = v.y; return cpv; };
    inline cpVect*      toChipmunk( const std::vector<ci::Vec2f>& vec ){
        cpVect* array = new cpVect[ vec.size() ];
        for( size_t i = 0; i < vec.size(); i++ ) array[i] = toChipmunk( vec[i] );
        return array;
    }
    inline cpBB         toChipmunk( const ci::Rectf &r ){ return cpBBNew( r.getX1(), r.getY1(), r.getX2(), r.getY2() ); }
    
    class Body;
    
    class Shape {
    public:
        Shape( cpShape* shape );
        
		void setFriction( float friction );
		void setElasticity( float elasticity );
		void setCollisionType( cpCollisionType collisionType );
		
        void setBody( Body body );
        Body getBody() const;
        
		void* getUserData() const;
		void setUserData( void* userData );
        
        cpShape* getCpShape() const;
		operator cpShape*() const;
        
    protected:
        Shape();
		Shape( Body body, cpShape* shape );
        
        struct Obj {
            Obj();
            Obj( cpShape* shape );
            ~Obj();
            
            bool        mAutoRelease;
            cpShape*    mShape;
        };
        
        std::shared_ptr<Obj>	mObj;
        Body*                   mBody;
        
    public:
        //@{
        //! Emulates shared_ptr-like behavior
        typedef std::shared_ptr<Obj> Shape::*unspecified_bool_type;
        operator unspecified_bool_type() const { return ( mObj.get() == 0 ) ? 0 : &Shape::mObj; }
        void reset() { mObj.reset(); }
        bool operator== (const Shape &shape ){ return mObj == shape.mObj; }
        
        //@}
        
    };
    
    class CircleShape : public Shape {
    public:
        CircleShape( Body body, float radius, ci::Vec2f offset = ci::Vec2f::zero() );
    };
    
    class BoxShape : public Shape {
    public:
        BoxShape( Body body, float width, float height );
        BoxShape( Body body, ci::Rectf rectangle );
    };
    
    class PolyShape : public Shape {
    public:
		PolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset = ci::Vec2f::zero() );
    };
    
    class SegmentShape : public Shape {
    public:
		SegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius = 1.0f );
    };
    
    
    class Body {
    public:
        Body( float mass = 1.0f, float moment = 1.0f );
        Body( ci::Vec2f position, float mass = 1.0f, float moment = 1.0f );
        Body( cpBody* body );
        
		ci::Vec2f getPosition() const;
		void setPosition( ci::Vec2f position );
        void setAngle( float angle );
        
		ci::Vec2f getVelocity() const;
		void setVelocity( ci::Vec2f velocity);
        
        void setMass( float mass );
        void setMoment( float moment );
        
		void* getUserData() const;
		void setUserData( void* );
        
        cpBody* getCpBody();
		operator cpBody*() const;
        
        void resetForces();
        void applyForce( ci::Vec2f force, ci::Vec2f point );
        void applyImpulse( ci::Vec2f impulse, ci::Vec2f point );
        
        ci::Vec2f localToWorld( ci::Vec2f local );
        ci::Vec2f worldToLocal( ci::Vec2f world );
        
    protected:
        struct Obj {
            Obj( cpBody* body );
            Obj( float mass, float moment );
            ~Obj();
            
            bool    mAutoRelease;
            cpBody* mBody;
        };
        
        std::shared_ptr< Obj >	mObj;
        std::vector< Shape >    mShapes;
        
    public:
        //@{
        //! Emulates shared_ptr-like behavior
        typedef std::shared_ptr<Obj> Body::*unspecified_bool_type;
        operator unspecified_bool_type() const { return ( mObj.get() == 0 ) ? 0 : &Body::mObj; }
        void reset() { mObj.reset(); }
        bool operator== (const Body &body ){ return mObj == body.mObj; }
        //@}
        
    };
    
    class Constraint {
    public:
        Constraint( cpConstraint* constraint );
        
        cpConstraint* getCpConstraint() const;
		operator cpConstraint*() const;
        
    protected:
        Constraint();
        Constraint( Body &bodyA, Body &bodyB, cpConstraint* constraint );
        
        struct Obj {
            Obj();
            Obj( cpConstraint* constraint );
            ~Obj();
            
            bool            mAutoRelease;
            cpConstraint*   mConstraint;
        };
        
        std::shared_ptr< Obj >	mObj;
        Body*                   mBodyA;
        Body*                   mBodyB;
        
    public:
        //@{
        //! Emulates shared_ptr-like behavior
        typedef std::shared_ptr<Obj> Constraint::*unspecified_bool_type;
        operator unspecified_bool_type() const { return ( mObj.get() == 0 ) ? 0 : &Constraint::mObj; }
        void reset() { mObj.reset(); }
        bool operator== (const Constraint &constraint ){ return mObj == constraint.mObj; }
        //@}
    };
    
    class PinJoint : public Constraint {
    public:
        PinJoint(){}
        PinJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB );
        
    };
    class SlideJoint : public Constraint {
    public:
        SlideJoint(){}
        SlideJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float min, float max );
        
    };
    class PivotJoint : public Constraint {
    public:
        PivotJoint(){}
        PivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f pivot );
        PivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB );
    };
    class GrooveJoint : public Constraint {
    public:
        GrooveJoint(){}
        GrooveJoint( Body &bodyA, Body &bodyB, ci::Vec2f grooveA, ci::Vec2f grooveB, ci::Vec2f anchorB );
        
    };
    class RotaryLimitJoint : public Constraint {
    public:
        RotaryLimitJoint(){}
        RotaryLimitJoint( Body &bodyA, Body &bodyB, float min, float max );
        
    };
    class RatchetJoint : public Constraint {
    public:
        RatchetJoint(){}
        RatchetJoint( Body &bodyA, Body &bodyB, float phase, float ratchet );
        
    };
    class GearJoint : public Constraint {
    public:
        GearJoint(){}
        GearJoint( Body &bodyA, Body &bodyB, float phase, float ratio );
        
    };
    
    
    class DampedSpring : public Constraint {
    public:
        DampedSpring(){}
        DampedSpring( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float restLength, float stiffness, float damping );
        
    };
    class DampedRotarySpring : public Constraint {
    public:
        DampedRotarySpring(){}
        DampedRotarySpring( Body &bodyA, Body &bodyB, float restAngle, float stiffness, float damping );
        
    };
    
    class SimpleMotor : public Constraint {
    public:
        SimpleMotor(){}
        SimpleMotor( Body &bodyA, Body &bodyB, float rate );
        
    };
    
    class Space {
    public:
        Space();
        
        void step( float dt = 1.0f / 60.0f );
        
        void drawShapes();
        void drawBoundingBoxes();
        void drawConstraints();
        void drawCollisionPoints();
        
        void setNumIterations( int iterations );
        void setSpatialHash( float dimension, int count );
        void setGravity( ci::Vec2f gravity );
        
        void addShape( const Shape &shape );
        
        CircleShape     addCircleShape( Body body, float radius, ci::Vec2f offset = ci::Vec2f::zero() );
        BoxShape        addBoxShape( Body body, float width, float height );
        BoxShape        addBoxShape( Body body, ci::Rectf rectangle );
		PolyShape       addPolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset = ci::Vec2f::zero() );
		SegmentShape    addSegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius = 1.0f );
        
        void addStaticShape( const Shape &shape );
        
        CircleShape     addStaticCircleShape( Body body, float radius, ci::Vec2f offset = ci::Vec2f::zero() );
        BoxShape        addStaticBoxShape( Body body, float width, float height );
        BoxShape        addStaticBoxShape( Body body, ci::Rectf rectangle );
		PolyShape       addStaticPolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset = ci::Vec2f::zero() );
		SegmentShape    addStaticSegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius = 1.0f );
        
        void addBody( Body &body );
        Body addBody( float mass = 1.0f, float moment = 1.0f );
        Body addBody( ci::Vec2f position, float mass = 1.0f, float moment = 1.0f );
        
        void addConstraint( const Constraint &constraint );
        
        PinJoint            addPinJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB );
        SlideJoint          addSlideJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float min, float max );
        PivotJoint          addPivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f pivot );
        PivotJoint          addPivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB );
        GrooveJoint         addGrooveJoint( Body &bodyA, Body &bodyB, ci::Vec2f grooveA, ci::Vec2f grooveB, ci::Vec2f anchorB );
        RotaryLimitJoint    addRotaryLimitJoint( Body &bodyA, Body &bodyB, float min, float max );
        RatchetJoint        addRatchetJoint( Body &bodyA, Body &bodyB, float phase, float ratchet );
        GearJoint           addGearJoint( Body &bodyA, Body &bodyB, float phase, float ratio );
        DampedSpring        addDampedSpring( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float restLength, float stiffness, float damping );
        DampedRotarySpring  addDampedRotarySpring( Body &bodyA, Body &bodyB, float restAngle, float stiffness, float damping );
        SimpleMotor         addSimpleMotor( Body &bodyA, Body &bodyB, float rate );
        
        void removeShape( Shape &shape );
        void removeBody( Body &body );
        void removeConstraint( Constraint &constraint );
        
        bool containsShape( Shape &shape );
        bool containsBody( Body &body );
        bool containsConstraint( Constraint &constraint );
        
        Shape pointQueryFirst( ci::Vec2f point, cpLayers layers, cpGroup group );
        
        inline ci::Vec2f localToWorld( ci::Vec2f local );
        inline ci::Vec2f worldToLocal( ci::Vec2f world );
        
        cpSpace* getCpSpace() const;
		operator cpSpace*() const;
    protected:
        struct Obj {
            Obj();
            ~Obj();
            
            cpSpace* mSpace;
        };
	
        std::shared_ptr< Obj >      mObj;
		std::vector< Shape >        mShapes;
		std::vector< Body >         mBodies;
        std::vector< Constraint >   mConstraints;

    public:
        //@{
        //! Emulates shared_ptr-like behavior
        typedef std::shared_ptr<Obj> Space::*unspecified_bool_type;
        operator unspecified_bool_type() const { return ( mObj.get() == 0 ) ? 0 : &Space::mObj; }
        void reset() { mObj.reset(); }
        //@}
    };
};