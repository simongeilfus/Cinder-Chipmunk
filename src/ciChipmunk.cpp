#include "ciChipmunk.h"
#include "cinder/gl/gl.h"
#include "chipmunk_private.h"
#include "boost/assign/list_of.hpp"

namespace cp {
    
    
    Shape::Shape()
    {
        mObj = std::shared_ptr<Obj>( new Obj() );
    }
    
    Shape::Shape( Body body, cpShape* shape )
    {
        mObj = std::shared_ptr<Obj>( new Obj() );
        mObj->mShape = shape;
        mBody = &body;
    }
    Shape::Shape( cpShape* shape )
    {
        if( shape ){
            mObj    = std::shared_ptr<Obj>( new Obj( shape ) );
            mBody   = new Body( shape->body );
        }
    }
    
    Shape::Obj::Obj() : mAutoRelease( true )
    {
        
    }
    Shape::Obj::Obj( cpShape* shape ) :
        mShape( shape ), mAutoRelease( false )
    {
        
    }
    Shape::Obj::~Obj()
    {
		if( mAutoRelease ) cpShapeFree( mShape );
    }
    
    
    void Shape::setFriction( float friction )
    {
		cpShapeSetFriction( mObj->mShape, friction );
    }
    void Shape::setElasticity( float elasticity )
    {
		cpShapeSetElasticity( mObj->mShape, elasticity );
        
    }
    void Shape::setCollisionType( cpCollisionType collisionType )
    {
		cpShapeSetCollisionType( mObj->mShape, collisionType );
    }
    void Shape::setUserData( void* userData )
    {
		cpShapeSetUserData( mObj->mShape, userData );
    }
    void Shape::setBody( Body body )
    {
		cpShapeSetBody( mObj->mShape, body.getCpBody() );
		mBody = &body;
    }
    Body Shape::getBody() const
    {
        return *mBody;
    }
    
    void* Shape::getUserData() const
    {
		return cpShapeGetUserData( mObj->mShape );
    }
    
    cpShape* Shape::getCpShape() const
    {
        return mObj->mShape;
    }
    Shape::operator cpShape*() const
    {
        return mObj->mShape;
    }
    
    //---------------------------------------------------------------------------------------------------------
    
    CircleShape::CircleShape( Body body, float radius, ci::Vec2f offset ) :
        Shape( body, cpCircleShapeNew( body.getCpBody(), radius, toChipmunk( offset ) ) )
    {
    }
    
    BoxShape::BoxShape( Body body, float width, float height ) :
        Shape( body, cpBoxShapeNew( body.getCpBody(), width, height ) )
    {
    }
    
    BoxShape::BoxShape( Body body, ci::Rectf rectangle ) :
        Shape( body, cpBoxShapeNew2( body.getCpBody(), toChipmunk( rectangle ) ) )
    {
        
    }
    
    PolyShape::PolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset ) :
        Shape( body, cpPolyShapeNew( body.getCpBody(), vertices.size(), toChipmunk( vertices ), toChipmunk( offset ) ) )
    {
    }
    
    SegmentShape::SegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius ) :
        Shape( body, cpSegmentShapeNew( body.getCpBody(), toChipmunk( a ), toChipmunk( b ), radius ) )
    {
    }
    
    //---------------------------------------------------------------------------------------------------------
    
    
    Body::Body( float mass, float moment ) :
        mObj( std::shared_ptr<Obj>( new Obj( mass, moment ) ) )
    {
    }
    
    Body::Body( ci::Vec2f position, float mass, float moment ) :
        mObj( std::shared_ptr<Obj>( new Obj( mass, moment ) ) )
    {
        setPosition( position );
    }
    Body::Body( cpBody* body ) :
        mObj( std::shared_ptr<Obj>( new Obj( body ) ) )
    {
    }
    
    Body::Obj::Obj( float mass, float moment ) :
        mBody( cpBodyNew( mass, moment ) ), mAutoRelease( true )
    {
    }
    
    Body::Obj::Obj( cpBody* body ) :
        mBody( body ), mAutoRelease( false )
    {
    }
    
    Body::Obj::~Obj()
    {
        if( mAutoRelease ) cpBodyFree( mBody );
    }
        
    ci::Vec2f Body::getPosition() const
    {
		return toCinder( cpBodyGetPos( mObj->mBody ) );
    }
    void Body::setPosition( ci::Vec2f position )
    {
		cpBodySetPos( mObj->mBody, toChipmunk( position ) );
    }
    void Body::setAngle( float angle )
    {
		cpBodySetAngle( mObj->mBody, angle );
    }
    
    ci::Vec2f Body::getVelocity() const
    {
		return toCinder( cpBodyGetVel( mObj->mBody ) );
    }
    void Body::setVelocity( ci::Vec2f velocity)
    {
		cpBodySetVel( mObj->mBody, toChipmunk( velocity ) );
    }
    
    
    void Body::setMass( float mass )
    {
		cpBodySetMass( mObj->mBody, mass );
    }
    void Body::setMoment( float moment )
    {
		cpBodySetMoment( mObj->mBody, moment );
    }
    
    void* Body::getUserData() const
    {
		return cpBodyGetUserData( mObj->mBody );
    }
    void Body::setUserData( void* userData )
    {
		cpBodySetUserData( mObj->mBody, userData );
    }
    
    
    cpBody* Body::getCpBody()
    {
        return mObj->mBody;
    }
    
    Body::operator cpBody*() const
    {
        return mObj->mBody;
    }
    
    void Body::resetForces()
    {
        cpBodyResetForces( mObj->mBody );
    }
    void Body::applyForce( ci::Vec2f force, ci::Vec2f point )
    {
        cpBodyApplyForce( mObj->mBody, toChipmunk( force ), toChipmunk( point ) );
    }
    void Body::applyImpulse( ci::Vec2f impulse, ci::Vec2f point )
    {
        cpBodyApplyImpulse( mObj->mBody, toChipmunk( impulse ), toChipmunk( point ) );
    }
    
    
    ci::Vec2f Body::localToWorld( ci::Vec2f local )
    {
        return toCinder( cpBodyLocal2World( mObj->mBody, toChipmunk( local ) ) );
    }
    ci::Vec2f Body::worldToLocal( ci::Vec2f world )
    {
        return toCinder( cpBodyWorld2Local( mObj->mBody, toChipmunk( world ) ) );
    }
    
    //---------------------------------------------------------------------------------------------------------
    
    
    Constraint::Constraint( cpConstraint* constraint ) {
        if( constraint ) {
            mObj = std::shared_ptr<Obj>( new Obj( constraint ) );
        }
    }
    
    Constraint::Constraint() {}
    Constraint::Constraint( Body &bodyA, Body &bodyB, cpConstraint* constraint ) :
        mObj( std::shared_ptr<Obj>( new Obj() ) ), mBodyA( &bodyA ), mBodyB( &bodyB ) {
            mObj->mConstraint = constraint;
        }
    
    Constraint::Obj::Obj() : mAutoRelease(true) {}
    Constraint::Obj::Obj( cpConstraint* constraint ) : mConstraint( constraint ), mAutoRelease(false)  {}
    Constraint::Obj::~Obj(){
        cpConstraintFree( mConstraint );
    }
        
    cpConstraint* Constraint::getCpConstraint() const
    {
        return mObj->mConstraint;
    }
    Constraint::operator cpConstraint*() const
    {
        return mObj->mConstraint;
    }
    
    
    PinJoint::PinJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB )
        : Constraint( bodyA, bodyB, cpPinJointNew( bodyA, bodyB, toChipmunk( anchorA ), toChipmunk( anchorB ) ) )
    {
    }
    SlideJoint::SlideJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float min, float max )
        : Constraint( bodyA, bodyB, cpSlideJointNew( bodyA, bodyB, toChipmunk( anchorA ), toChipmunk( anchorB ), min, max ) )
    {
    }
    
    PivotJoint::PivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f pivot )
        : Constraint( bodyA, bodyB, cpPivotJointNew( bodyA, bodyB, toChipmunk( pivot ) ) )
    {
    }
    PivotJoint::PivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB )
        : Constraint( bodyA, bodyB, cpPivotJointNew2( bodyA, bodyB, toChipmunk( anchorA ), toChipmunk( anchorB ) ) )
    {
    }
    GrooveJoint::GrooveJoint( Body &bodyA, Body &bodyB, ci::Vec2f grooveA, ci::Vec2f grooveB, ci::Vec2f anchorB )
        : Constraint( bodyA, bodyB, cpGrooveJointNew( bodyA, bodyB, toChipmunk( grooveA ), toChipmunk( grooveB ), toChipmunk( anchorB ) ) )
    {
    }
    RotaryLimitJoint::RotaryLimitJoint( Body &bodyA, Body &bodyB, float min, float max )
        : Constraint( bodyA, bodyB, cpRotaryLimitJointNew( bodyA, bodyB, min, max ) )
    {
    }
    RatchetJoint::RatchetJoint( Body &bodyA, Body &bodyB, float phase, float ratchet )
        : Constraint( bodyA, bodyB, cpRatchetJointNew( bodyA, bodyB, phase, ratchet ) )
    {
    }
    GearJoint::GearJoint( Body &bodyA, Body &bodyB, float phase, float ratio )
        : Constraint( bodyA, bodyB, cpGearJointNew( bodyA, bodyB, phase, ratio ) )
    {
    }
    DampedSpring::DampedSpring( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float restLength, float stiffness, float damping )
        : Constraint( bodyA, bodyB, cpDampedSpringNew( bodyA, bodyB, toChipmunk( anchorA ), toChipmunk( anchorB ), restLength, stiffness, damping ) )
    {
    }
    DampedRotarySpring::DampedRotarySpring( Body &bodyA, Body &bodyB, float restAngle, float stiffness, float damping )
        : Constraint( bodyA, bodyB, cpDampedRotarySpringNew( bodyA, bodyB, restAngle, stiffness, damping ) )
    {
    }
    SimpleMotor::SimpleMotor( Body &bodyA, Body &bodyB, float rate )
        : Constraint( bodyA, bodyB, cpSimpleMotorNew( bodyA, bodyB, rate ) )
    {
    }
    
    
    //---------------------------------------------------------------------------------------------------------
    
    static void shapeFreeWrap(cpSpace *space, cpShape *shape, void *unused){
        cpSpaceRemoveShape(space, shape);
        cpShapeFree(shape);
    }
    
    static void postShapeFree(cpShape *shape, cpSpace *space){
        cpSpaceAddPostStepCallback(space, (cpPostStepFunc)shapeFreeWrap, shape, NULL);
    }
    
    static void constraintFreeWrap(cpSpace *space, cpConstraint *constraint, void *unused){
        cpSpaceRemoveConstraint(space, constraint);
        cpConstraintFree(constraint);
    }
    
    static void postConstraintFree(cpConstraint *constraint, cpSpace *space){
        cpSpaceAddPostStepCallback(space, (cpPostStepFunc)constraintFreeWrap, constraint, NULL);
    }
    
    static void bodyFreeWrap(cpSpace *space, cpBody *body, void *unused){
        cpSpaceRemoveBody(space, body);
        cpBodyFree(body);
    }
    
    static void postBodyFree(cpBody *body, cpSpace *space){
        cpSpaceAddPostStepCallback(space, (cpPostStepFunc)bodyFreeWrap, body, NULL);
    }
    
    //---------------------------------------------------------------------------------------------------------

    Space::Space()
    {
        mObj = std::shared_ptr<Obj>( new Obj() );
    }
    
    Space::Obj::Obj()
    {
        mSpace = cpSpaceNew();
        cpSpaceSetIterations( mSpace, 20 );
    }
    Space::Obj::~Obj()
    {
        //cpSpaceEachShape( mSpace, (cpSpaceShapeIteratorFunc)postShapeFree, mSpace );
        //cpSpaceEachConstraint( mSpace, (cpSpaceConstraintIteratorFunc)postConstraintFree, mSpace );
        //cpSpaceEachBody( mSpace, (cpSpaceBodyIteratorFunc)postBodyFree, mSpace );
        
        cpSpaceFree( mSpace );
    }
    
    void Space::step( float dt )
    {
        cpSpaceStep( mObj->mSpace, dt );
    }
    
    static void drawShapeWrap( cpShape *shape, void *unused )
    {
        switch( shape->klass->type ){
            case CP_CIRCLE_SHAPE: {
                cpCircleShape *circle = (cpCircleShape *)shape;
                ci::gl::drawStrokedCircle( cp::toCinder( circle->tc ), circle->r );
                break;
            }
            case CP_SEGMENT_SHAPE: {
                cpSegmentShape *seg = (cpSegmentShape *)shape;
                ci::gl::drawLine( cp::toCinder( seg->ta ), cp::toCinder( seg->tb ) );
                break;
            }
            case CP_POLY_SHAPE: {
                cpPolyShape *poly = (cpPolyShape *)shape;
                glBegin( GL_POLYGON );
                for( int i = 0; i < poly->numVerts; i++ ){
                    glVertex2f( poly->tVerts[i].x, poly->tVerts[i].y );
                }
                glEnd();
                break;
            }
            default: break;
        }
    }
    
    void Space::drawShapes()
    {
        cpSpaceEachShape( mObj->mSpace, drawShapeWrap, NULL );
    }
    
    static void drawBoundingBoxWrap( cpShape *shape, void *unused )
    {
        ci::Rectf rect = toCinder( shape->bb );
        ci::gl::drawStrokedRect( rect );
    }
    void Space::drawBoundingBoxes()
    {
        cpSpaceEachShape( mObj->mSpace, drawBoundingBoxWrap, NULL );
    }
    
    static std::vector<ci::Vec2f> springPolyline = boost::assign::list_of( ci::Vec2f( 0.00f, 0.0f ) )( ci::Vec2f( 0.20f, 0.0f ) )( ci::Vec2f( 0.25f, 3.0f ) )( ci::Vec2f( 0.30f,-6.0f ) )( ci::Vec2f( 0.35f, 6.0f ) )( ci::Vec2f( 0.40f,-6.0f ) )( ci::Vec2f( 0.45f, 6.0f ) )( ci::Vec2f( 0.50f,-6.0f ) )( ci::Vec2f( 0.55f, 6.0f ) )( ci::Vec2f( 0.60f,-6.0f ) )( ci::Vec2f( 0.65f, 6.0f ) )( ci::Vec2f( 0.70f,-3.0f ) )( ci::Vec2f( 0.75f, 6.0f ) )( ci::Vec2f( 0.80f, 0.0f ) )( ci::Vec2f( 1.00f, 0.0f ) );
    
    static void drawConstraintWrap( cpConstraint *constraint, void *unused )
    {
        cpBody *body_a = constraint->a;
        cpBody *body_b = constraint->b;
        glPointSize(4.0f);
        const cpConstraintClass *klass = constraint->klass;
        if(klass == cpPinJointGetClass()){
            cpPinJoint *joint = (cpPinJoint *)constraint;
            
            cpVect a = cpvadd(body_a->p, cpvrotate(joint->anchr1, body_a->rot));
            cpVect b = cpvadd(body_b->p, cpvrotate(joint->anchr2, body_b->rot));
            
            glBegin( GL_POINTS );
            glVertex2f( toCinder( a ) );
            glVertex2f( toCinder( b ) );
            glEnd();
            ci::gl::drawLine( toCinder( a ), toCinder( b ) );
        } else if(klass == cpSlideJointGetClass()){
            cpSlideJoint *joint = (cpSlideJoint *)constraint;
            
            cpVect a = cpvadd(body_a->p, cpvrotate(joint->anchr1, body_a->rot));
            cpVect b = cpvadd(body_b->p, cpvrotate(joint->anchr2, body_b->rot));
            
            glBegin( GL_POINTS );
            glVertex2f( toCinder( a ) );
            glVertex2f( toCinder( b ) );
            glEnd();
            ci::gl::drawLine( toCinder( a ), toCinder( b ) );
        } else if(klass == cpPivotJointGetClass()){
            cpPivotJoint *joint = (cpPivotJoint *)constraint;
            
            cpVect a = cpvadd(body_a->p, cpvrotate(joint->anchr1, body_a->rot));
            cpVect b = cpvadd(body_b->p, cpvrotate(joint->anchr2, body_b->rot));
            
            ci::gl::drawLine( toCinder( a ), toCinder( b ) );
            glBegin( GL_POINTS );
            glVertex2f( toCinder( a ) );
            glVertex2f( toCinder( b ) );
            glEnd();
        } else if(klass == cpGrooveJointGetClass()){
            cpGrooveJoint *joint = (cpGrooveJoint *)constraint;
            
            cpVect a = cpvadd(body_a->p, cpvrotate(joint->grv_a, body_a->rot));
            cpVect b = cpvadd(body_a->p, cpvrotate(joint->grv_b, body_a->rot));
            cpVect c = cpvadd(body_b->p, cpvrotate(joint->anchr2, body_b->rot));
            
            glBegin( GL_POINTS );
            glVertex2f( toCinder( a ) );
            glVertex2f( toCinder( b ) );
            glVertex2f( toCinder( c ) );
            glEnd();
            ci::gl::drawLine( toCinder( a ), toCinder( b ) );
        } else if(klass == cpDampedSpringGetClass()){
            cpDampedSpring *spring = (cpDampedSpring *)constraint;
            cpVect a = cpvadd(body_a->p, cpvrotate(spring->anchr1, body_a->rot));
            cpVect b = cpvadd(body_b->p, cpvrotate(spring->anchr2, body_b->rot));
            
            glBegin( GL_POINTS );
            glVertex2f( toCinder( a ) );
            glVertex2f( toCinder( b ) );
            glEnd();
            
            cpVect delta = cpvsub(b, a);
            ci::gl::pushModelView();
            GLfloat x = a.x;
            GLfloat y = a.y;
            GLfloat cos = delta.x;
            GLfloat sin = delta.y;
            GLfloat s = 1.0f/cpvlength(delta);
                
            const GLfloat matrix[] = {
                cos,    sin, 0.0f, 0.0f,
                -sin*s,  cos*s, 0.0f, 0.0f,
                0.0f,   0.0f, 1.0f, 0.0f,
                x,      y, 0.0f, 1.0f,
            };
                
            ci::gl::multModelView( ci::Matrix44f( matrix ) );
            ci::gl::draw( springPolyline );
            ci::gl::popModelView();
        }
        glPointSize(1.0f);
    }
    void Space::drawConstraints()
    {
        cpSpaceEachConstraint( mObj->mSpace, drawConstraintWrap, NULL );
    }
    void Space::drawCollisionPoints()
    {
        
        cpArray *arbiters = mObj->mSpace->arbiters;
        
        glColor3f(1.0f, 0.0f, 0.0f);
        glPointSize( 4.0f );
        
        glBegin(GL_POINTS); {
            for(int i=0; i < arbiters->num; i++){
                cpArbiter *arb = (cpArbiter*)arbiters->arr[i];
                
                for(int j=0; j<arb->numContacts; j++){
                    cpVect v = arb->contacts[j].p;
                    glVertex2f(v.x, v.y);
                }
            }
        }
        glEnd();
    }
    
    void Space::setNumIterations( int iterations )
    {
        cpSpaceSetIterations( mObj->mSpace, iterations );
    }
    void Space::setSpatialHash( float dimension, int count )
    {
        cpSpaceUseSpatialHash( mObj->mSpace, dimension, count );
    }
    void Space::setGravity( ci::Vec2f gravity )
    {
        cpSpaceSetGravity( mObj->mSpace, toChipmunk( gravity ) );
    }
    
    void Space::addShape( const Shape &shape )
    {
		cpSpaceAddShape( mObj->mSpace, shape.getCpShape() );
        mShapes.push_back( shape );
    }
    CircleShape Space::addCircleShape( Body body, float radius, ci::Vec2f offset )
    {
        CircleShape shape( body, radius, offset );
        addShape( shape );
        return shape;
    }
    BoxShape Space::addBoxShape( Body body, float width, float height )
    {
        BoxShape shape( body, width, height );
        addShape( shape );
        return shape;
    }
    BoxShape Space::addBoxShape( Body body, ci::Rectf rectangle )
    {
        BoxShape shape( body, rectangle );
        addShape( shape );
        return shape;
    }
    PolyShape Space::addPolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset )
    {
        PolyShape shape( body, vertices, offset );
        addShape( shape );
        return shape;
    }
    SegmentShape Space::addSegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius )
    {
        SegmentShape shape( body, a, b, radius );
        addShape( shape );
        return shape;
    }
    
    
    void Space::addStaticShape( const Shape &shape )
    {
        cpSpaceAddStaticShape( mObj->mSpace, shape.getCpShape() );
        mShapes.push_back( shape );
    }
    CircleShape Space::addStaticCircleShape( Body body, float radius, ci::Vec2f offset )
    {
        CircleShape shape( body, radius, offset );
        addStaticShape( shape );
        return shape;
    }
    BoxShape Space::addStaticBoxShape( Body body, float width, float height )
    {
        BoxShape shape( body, width, height );
        addStaticShape( shape );
        return shape;
    }
    BoxShape Space::addStaticBoxShape( Body body, ci::Rectf rectangle )
    {
        BoxShape shape( body, rectangle );
        addStaticShape( shape );
        return shape;
    }
    PolyShape Space::addStaticPolyShape( Body body, const std::vector<ci::Vec2f>& vertices, ci::Vec2f offset )
    {
        PolyShape shape( body, vertices, offset );
        addStaticShape( shape );
        return shape;
    }
    SegmentShape Space::addStaticSegmentShape( Body body, ci::Vec2f a, ci::Vec2f b, float radius )
    {
        SegmentShape shape( body, a, b, radius );
        addStaticShape( shape );
        return shape;
    }
    
    void Space::addBody( Body &body )
    {
		cpSpaceAddBody( mObj->mSpace, body.getCpBody() );
		mBodies.push_back( body );
    }
    
    Body Space::addBody( float mass, float moment )
    {
        Body body( mass, moment );
        addBody( body );
        return body;
    }
    Body Space::addBody( ci::Vec2f position, float mass, float moment )
    {
        Body body( position, mass, moment );
        addBody( body );
        return body;
    }
    
    void Space::addConstraint( const Constraint &constraint )
    {
        cpSpaceAddConstraint( mObj->mSpace, constraint.getCpConstraint() );
        mConstraints.push_back( constraint );
    }
        
    
    
    PinJoint Space::addPinJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB )
    {
        PinJoint constraint( bodyA, bodyB, anchorA, anchorB );
        addConstraint( constraint );
        return constraint;
    }
    SlideJoint Space::addSlideJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float min, float max )
    {
        SlideJoint constraint( bodyA, bodyB, anchorA, anchorB, min, max );
        addConstraint( constraint );
        return constraint;
    }
    PivotJoint Space::addPivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f pivot )
    {
        PivotJoint constraint( bodyA, bodyB, pivot );
        addConstraint( constraint );
        return constraint;
    }
    PivotJoint Space::addPivotJoint( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB )
    {
        PivotJoint constraint( bodyA, bodyB, anchorA, anchorB );
        addConstraint( constraint );
        return constraint;
    }
    GrooveJoint Space::addGrooveJoint( Body &bodyA, Body &bodyB, ci::Vec2f grooveA, ci::Vec2f grooveB, ci::Vec2f anchorB )
    {
        GrooveJoint constraint( bodyA, bodyB, grooveA, grooveB, anchorB );
        addConstraint( constraint );
        return constraint;
    }
    RotaryLimitJoint Space::addRotaryLimitJoint( Body &bodyA, Body &bodyB, float min, float max )
    {
        RotaryLimitJoint constraint( bodyA, bodyB, min, max );
        addConstraint( constraint );
        return constraint;
    }
    RatchetJoint Space::addRatchetJoint( Body &bodyA, Body &bodyB, float phase, float ratchet )
    {
        RatchetJoint constraint( bodyA, bodyB, phase, ratchet );
        addConstraint( constraint );
        return constraint;
    }
    GearJoint Space::addGearJoint( Body &bodyA, Body &bodyB, float phase, float ratio )
    {
        GearJoint constraint( bodyA, bodyB, phase, ratio );
        addConstraint( constraint );
        return constraint;
    }
    DampedSpring Space::addDampedSpring( Body &bodyA, Body &bodyB, ci::Vec2f anchorA, ci::Vec2f anchorB, float restLength, float stiffness, float damping )
    {
        DampedSpring constraint( bodyA, bodyB, anchorA, anchorB, restLength, stiffness, damping );
        addConstraint( constraint );
        return constraint;
    }
    DampedRotarySpring Space::addDampedRotarySpring( Body &bodyA, Body &bodyB, float restAngle, float stiffness, float damping )
    {
        DampedRotarySpring constraint( bodyA, bodyB, restAngle, stiffness, damping );
        addConstraint( constraint );
        return constraint;
    }
    SimpleMotor Space::addSimpleMotor( Body &bodyA, Body &bodyB, float rate )
    {
        SimpleMotor constraint( bodyA, bodyB, rate );
        addConstraint( constraint );
        return constraint;
    }
    
    void Space::removeShape( Shape &shape )
    {
        cpSpaceRemoveShape( mObj->mSpace, shape );
        mShapes.erase(std::find( mShapes.begin(), mShapes.end(), shape ) );
        shape.reset();
    }
    void Space::removeBody( Body &body )
    {
        cpSpaceRemoveBody( mObj->mSpace, body );
		mBodies.erase( std::find( mBodies.begin(), mBodies.end(), body ) );
        body.reset();
    }
    void Space::removeConstraint( Constraint &constraint )
    {
        cpSpaceRemoveConstraint( mObj->mSpace, constraint );
		mConstraints.erase( std::find( mConstraints.begin(), mConstraints.end(), constraint ) );
        constraint.reset();
    }
        
    bool Space::containsShape( Shape &shape )
    {
        return cpSpaceContainsShape( mObj->mSpace, shape.getCpShape() );
    }
    bool Space::containsBody( Body &body )
    {
        return cpSpaceContainsBody( mObj->mSpace, body.getCpBody() );
    }
    bool Space::containsConstraint( Constraint &constraint )
    {
        return cpSpaceContainsConstraint( mObj->mSpace, constraint.getCpConstraint() );
    }
    
    
    Shape Space::pointQueryFirst( ci::Vec2f point, cpLayers layers, cpGroup group )
    {
        return cpSpacePointQueryFirst( mObj->mSpace, toChipmunk( point ), layers, group );
    }
    
    
    cpSpace* Space::getCpSpace() const
    {
        return mObj->mSpace;
    }
    Space::operator cpSpace*() const
    {
        return mObj->mSpace;
    }
    
        

            
};