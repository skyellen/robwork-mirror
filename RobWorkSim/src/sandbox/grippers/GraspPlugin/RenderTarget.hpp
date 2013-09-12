/**
 * @file RenderTarget.hpp
 * @brief Taken from GTaskVisPlugin
 */

#pragma once

#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/task/GraspTask.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rw/graphics/Render.hpp>



class RenderTargets: public rw::graphics::Render
{
	public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<RenderTargets> Ptr;

		struct Target {
			GLfloat color[4];
			Transform3D<> trans;
			double scale;
			bool enabled;
			rwlibs::task::GraspSubTask *ctask;
			rwlibs::task::GraspTarget ctarget;
		};

		RenderTargets():_size(-0.02), _zoffset(0.0){
			rw::geometry::Box box(_size/8,_size/2,_size/2);
			mesh = box.getTriMesh();
		};

		/* Functions inherited from Render */
		/**
		 * @copydoc Render::draw
		 */
		void draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const {
			glPushMatrix();
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
			glDisable(GL_LIGHTING);
			BOOST_FOREACH(Target target, _targets){
				//if(!target.enabled)
				//    continue;
				const Vector3D<> &zoffset = _zoffset*( target.trans.R()*Vector3D<>::z() );
				const Vector3D<> &p = target.trans.P();
				const Vector3D<> &pn = p+target.trans.R()*Vector3D<>::z()*_size + zoffset;
				const Vector3D<> &pc = p+(target.trans.R()*Vector3D<>::x()*_size/8);
				glBegin(GL_LINES);
				glColor3fv(target.color);
				glVertex3f(p[0],p[1],p[2]);
				glVertex3f(pn[0],pn[1],pn[2]);
				glColor3f(0.0f,0.0f,0.0f);
				glVertex3f(p[0],p[1],p[2]);
				glVertex3f(pc[0],pc[1],pc[2]);
				glEnd();

				glBegin(GL_TRIANGLES);
				glColor3fv(target.color);
				for(size_t i=0;i<mesh->size();i++){
					const rw::geometry::Triangle<> tri = mesh->getTriangle(i);
					rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[0]);
					rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[1]);
					rwlibs::opengl::DrawableUtil::drawGLVertex(pn+target.trans.R()*tri[2]);
				}
				glEnd();
			}
			glEnable(GL_LIGHTING);
			glPopMatrix();
		}

		void setTargets(std::vector<Target>& targets){
			_targets = targets;
		}

		const std::vector<Target>& getTargets(){ return _targets; }

		void setZOffset(float offset){
			_zoffset = offset;
		}
		
	private:
		float _size;
		float _zoffset;
		std::vector<Target> _targets;
		rw::geometry::TriMesh::Ptr mesh;
};
