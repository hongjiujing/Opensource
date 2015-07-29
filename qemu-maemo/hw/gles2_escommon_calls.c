static unsigned gles2_glTexParameterCount(TGLenum pname)
{
    unsigned count;

    switch(pname) {
        case GL_TEXTURE_MIN_FILTER: count = 1; break;
        case GL_TEXTURE_MAG_FILTER: count = 1; break;
        case GL_TEXTURE_WRAP_S: count = 1; break;
        case GL_TEXTURE_WRAP_T: count = 1; break;
        default:
            GLES2_PRINT("ERROR: Unknown texture parameter 0x%x!\n", pname);
            count = 1;
            break;
    }

    return count;
}

static void gles2_TransferArrays(gles2_State *s, gles2_Context *c,
    TGLint first, TGLsizei count)
{
    int i;
    for(i = 0; i < c->narrays; ++i) {
        gles2_Array*const va = c->arrays + i;

        //Do not transfer data if array is disabled or if vbo based
        if(!va->enabled || va->buffer) {
            continue;
        }
        unsigned esize = 1;
        switch (va->type) {
            case GL_BYTE:
            case GL_UNSIGNED_BYTE:  esize = 1; break;
            case GL_SHORT:
            case GL_UNSIGNED_SHORT: esize = 2; break;
            case GL_FIXED:
            case GL_FLOAT:          esize = 4; break;
        }
        if(!va->stride) {
            va->stride = va->size*esize;
        }

        if(va->ptr) {
            free(va->ptr);
        }

        va->delta = va->stride*first;
        unsigned nbytes = esize*count*va->size;
        va->ptr = malloc(nbytes);

        unsigned int j;
        for (j = 0; j < count; ++j) {
            signed k;
            for (k = 0; k < va->size; ++k) {
                switch (esize) {
                    case 1:
                        ((TGLubyte*)va->ptr)[j*va->size + k] =
                            gles2_get_byte(s, va->tptr + va->stride*(first + j)
                            + k*sizeof(TGLubyte));
                        break;
                    case 2:
                        ((TGLushort*)va->ptr)[j*va->size + k] =
                            gles2_get_word(s, va->tptr + va->stride*(first + j)
                            + k*sizeof(TGLushort));
                        break;
                    case 4:
                        if(va->type == GL_FLOAT) {
                            ((TGLfloat*)va->ptr)[j*va->size + k] =
                                gles2_get_float(s, va->tptr
                                + va->stride*(first + j)
                                + k*sizeof(TGLfloat));
                        } else {
                            ((TGLuint*)va->ptr)[j*va->size + k] =
                                gles2_get_dword(s, va->tptr
                                + va->stride*(first + j)
                                + k*sizeof(TGLuint));
                        }
                        break;
                }
            }
        }
        //clear error
        hgl.glGetError();
        va->apply(va);
    }
}

static void gles2_update_ebo(gles2_Context* ctx, GLuint buffer, void* data, GLsizeiptr size, GLintptr offset, int isupdate)
{
    gles2_ebo* ebo = gles2_ebo_find(buffer, ctx->ebo_list);
    if(!ebo) {
        ebo = gles2_ebo_add(buffer, ctx->ebo_list);
    }
    //if whole data is being recreated  (i.e. glBufferData)
    if(!isupdate) {
        if(ebo->data)
            free(ebo->data);
        ebo->data = data;
        ebo->size = size;
    //otherwise update the existing data
    } else {
        int i;
        //check just in case
        if(offset+size <= ebo->size) {
            for(i=0;i<size; i++) {
                ((char*)ebo->data)[i+offset]=((char*)data)[i];
            }
        }
    }
    //just being paranoid about type sizes
    int i, count = ebo->size/sizeof(GLubyte), q = sizeof(GLushort)/sizeof(GLubyte);
    GLushort s, s_min, s_max;
    GLubyte b, b_min, b_max;

    b_min = (1<<(sizeof(GLubyte)*8))-1;
    b_max = 0;
    s_min = (1<<(sizeof(GLushort)*8))-1;
    s_max = 0;

    //find the min/max for both types since we don't really know which at this point
    for(i=0; i< count; i++) {
        b = ((GLubyte*)ebo->data)[i];
        b_min = (b < b_min)?b:b_min;
        b_max = (b > b_max)?b:b_max;

        if(!i%q) {
            s = ((GLushort*)ebo->data)[i];
            s_min = (s < s_min)?s:s_min;
            s_max = (s > s_max)?s:s_max;
        }
    }

    ebo->b_min = b_min;
    ebo->b_max = b_max;
    ebo->s_min = s_min;
    ebo->s_max = s_max;
}

GLES2_CB(glActiveTexture)
{
    GLES2_ARG(TGLenum, texture);
    GLES2_BARRIER_ARG_NORET;

    hgl.glActiveTexture(texture);
}

GLES2_CB(glBindBuffer)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLuint, buffer);
    GLES2_BARRIER_ARG_NORET;

    hgl.glBindBuffer(target, buffer);
}

GLES2_CB(glBindTexture)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLuint, texture);
    GLES2_BARRIER_ARG_NORET;

    hgl.glBindTexture(target, texture);
}

GLES2_CB(glBlendFunc)
{
    GLES2_ARG(TGLenum, sfactor);
    GLES2_ARG(TGLenum, dfactor);
    GLES2_BARRIER_ARG_NORET;

    hgl.glBlendFunc(sfactor, dfactor);
}

GLES2_CB(glBufferData)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLsizei, size);
    GLES2_ARG(Tptr, datap);
    GLES2_ARG(TGLenum, usage);

    void *data = NULL;
    if (datap && size > 0) {
        data = malloc(size);
        gles2_transfer(s, datap, size, data, 0);
    }
    GLES2_BARRIER_ARG_NORET;

    GLuint buffer = 0;
    hgl.glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, (GLint *)&buffer);

    if(buffer) {
        gles2_Context * ctx = c->context[context_index];
        gles2_update_ebo(ctx, buffer, data, size, 0, 0);
    }

    hgl.glBufferData(target, size, data, usage);
    //if ebo is binded then copy of data will be kept
    if (!buffer && data) {
        free(data);
    }
}

GLES2_CB(glBufferSubData)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, offset);
    GLES2_ARG(TGLsizei, size);
    GLES2_ARG(Tptr, datap);

    void *data = NULL;
    if (datap && size > 0) {
        data = malloc(size);
        gles2_transfer(s, datap, size, data, 0);
    }
    GLES2_BARRIER_ARG_NORET;

    GLuint buffer = 0;
    hgl.glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, (GLint *)&buffer);
    gles2_Context * ctx = c->context[context_index];

    if(buffer && gles2_ebo_find(buffer, ctx->ebo_list)) {
        gles2_update_ebo(ctx, buffer, data, size, offset, 1);
    }

    hgl.glBufferSubData(target, offset, size, data);
    if (data) {
        free(data);
    }
}

GLES2_CB(glClear)
{
    GLES2_ARG(TGLbitfield, mask);
    GLES2_BARRIER_ARG_NORET;

    hgl.glClear(mask);
}

GLES2_CB(glClearColor)
{
    GLES2_ARG(TGLclampf, red);
    GLES2_ARG(TGLclampf, green);
    GLES2_ARG(TGLclampf, blue);
    GLES2_ARG(TGLclampf, alpha);
    GLES2_BARRIER_ARG_NORET;

    hgl.glClearColor(red, green, blue, alpha);
}

GLES2_CB(glClearDepthf)
{
    GLES2_ARG(TGLclampf, depth);
    GLES2_BARRIER_ARG_NORET;

    hgl.glClearDepthf(depth);
}

GLES2_CB(glClearStencil)
{
    GLES2_ARG(TGLint, s_);
    GLES2_BARRIER_ARG_NORET;

    hgl.glClearStencil(s_);
}

GLES2_CB(glColorMask)
{
    GLES2_ARG(TGLboolean, red);
    GLES2_ARG(TGLboolean, green);
    GLES2_ARG(TGLboolean, blue);
    GLES2_ARG(TGLboolean, alpha);
    GLES2_BARRIER_ARG_NORET;

    hgl.glColorMask(red, green, blue, alpha);
}

GLES2_CB(glCompressedTexImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLenum, internalformat);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLint, border);
    GLES2_ARG(TGLsizei, imageSize);
    GLES2_ARG(Tptr, data);

    GLES2_PRINT("Uploading compressed %dx%d image with size 0x%x and border %d...\n",
                width, height, imageSize, border);

    char *pixels = NULL;
    if (imageSize > 0 && data) { 
        char* pixels = malloc(imageSize);
        gles2_transfer(s, data, imageSize, pixels, 0);
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glCompressedTexImage2D(target, level, internalformat, width, height, border, imageSize, pixels);
    if (pixels) {
        free(pixels);
    }
}

GLES2_CB(glCompressedTexSubImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLint, xoffset);
    GLES2_ARG(TGLint, yoffset);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLenum, format);
    GLES2_ARG(TGLsizei, imageSize);
    GLES2_ARG(Tptr, data);

    GLES2_PRINT("Uploading compressed partial %dx%d image at %d,%d with size 0x%x...\n",
                width, height, xoffset, yoffset, imageSize);

    char* pixels = NULL;
    if (imageSize > 0 && data) {
        pixels = malloc(imageSize);
        gles2_transfer(s, data, imageSize, pixels, 0);
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glCompressedTexSubImage2D(target, level, xoffset, yoffset, width, height, format, imageSize, pixels);
    if (pixels) {
        free(pixels);
    }
}

GLES2_CB(glCopyTexImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLenum, internalformat);
    GLES2_ARG(TGLint, x);
    GLES2_ARG(TGLint, y);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLint, border);

    GLES2_BARRIER_ARG;

    hgl.glCopyTexImage2D(target, level, internalformat, x, y, width, height, border);

    GLES2_BARRIER_RET;
}

GLES2_CB(glCopyTexSubImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLint, xoffset);
    GLES2_ARG(TGLint, yoffset);
    GLES2_ARG(TGLint, x);
    GLES2_ARG(TGLint, y);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);

    GLES2_BARRIER_ARG;

    hgl.glCopyTexSubImage2D(target, level, xoffset, yoffset, x, y, width, height);

    GLES2_BARRIER_RET;
}

GLES2_CB(glCullFace)
{
    GLES2_ARG(TGLenum, mode);
    GLES2_BARRIER_ARG_NORET;

    hgl.glCullFace(mode);
}

GLES2_CB(glDeleteBuffers)
{
    GLES2_ARG(TGLsizei, n);
    GLES2_ARG(Tptr, buffersp);

    GLuint *buffers = NULL;
    if (n > 0) {
        buffers = malloc(n * sizeof(GLuint));
        GLsizei i;
        gles2_Context * ctx = c->context[context_index];
        for (i = 0; i < n; i++) {
            buffers[i] = gles2_get_TGLuint(s, buffersp + i * sizeof(TGLuint));
            gles2_ebo_remove(buffers[i], ctx->ebo_list);
        }
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glDeleteBuffers(n, buffers);
    if (buffers) {
        free(buffers);
    }
}

GLES2_CB(glDeleteTextures)
{
    GLES2_ARG(TGLsizei, n);
    GLES2_ARG(Tptr, texturesp);

    GLuint* textures = NULL;
    if (n > 0) {
        textures = malloc(sizeof(GLuint)*n);
        GLsizei i;
        for(i = 0; i < n; ++i) {
            textures[i] = gles2_get_TGLuint(s, texturesp + i*sizeof(TGLuint));
        }
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glDeleteTextures(n, textures);
    if (textures) {
        free(textures);
    }
}

GLES2_CB(glDepthFunc)
{
    GLES2_ARG(TGLenum, func);
    GLES2_BARRIER_ARG_NORET;

    hgl.glDepthFunc(func);
}

GLES2_CB(glDepthMask)
{
    GLES2_ARG(TGLboolean, flag);
    GLES2_BARRIER_ARG_NORET;

    hgl.glDepthMask(flag);
}

GLES2_CB(glDepthRangef)
{
    GLES2_ARG(TGLclampf, zNear);
    GLES2_ARG(TGLclampf, zFar);
    GLES2_BARRIER_ARG_NORET;

    hgl.glDepthRangef(zNear, zFar);
}

GLES2_CB(glDisable)
{
    GLES2_ARG(TGLenum, cap);
    GLES2_BARRIER_ARG;

    hgl.glDisable(cap);
    GLES2_BARRIER_RET;
}

GLES2_CB(glDrawArrays)
{
    GLES2_ARG(TGLenum, mode);
    GLES2_ARG(TGLint, first);
    GLES2_ARG(TGLsizei, count);

    if (first >=0 && count > 0) {
        gles2_Context * ctx = c->context[context_index];
        gles2_TransferArrays(s, ctx, first, count);
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glDrawArrays(mode, first, count);
}

GLES2_CB(glDrawElements)
{
    GLES2_ARG(TGLenum, mode);
    GLES2_ARG(TGLsizei, count);
    GLES2_ARG(TGLenum, type);
    GLES2_ARG(Tptr, indicesp);

    void *copied_indices = NULL;
    GLuint buffer = 0;
    int i, first = -1, last = -1;
    gles2_Context * ctx = c->context[context_index];
    hgl.glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, (GLint *)&buffer);
    if (!buffer) {
        GLint indice_size;
        switch (type) {
            case GL_UNSIGNED_BYTE: indice_size = sizeof(TGLubyte); break;
            case GL_UNSIGNED_SHORT: indice_size = sizeof(TGLushort); break;
            default:
                fprintf(stderr, "ERROR: Invalid type %d!\n", type);
                return;
        }
        if (count > 0) {
            copied_indices = malloc(indice_size * count);
            for (i = 0; i < count; i++) {
                TGLushort idx = 0;
                switch (type) {
                    case GL_UNSIGNED_BYTE:
                        idx = gles2_get_byte(s, indicesp++);
                        ((TGLubyte *)copied_indices)[i] = (TGLubyte)idx;
                        break;
                    case GL_UNSIGNED_SHORT:
                        idx = gles2_get_word(s, indicesp);
                        ((TGLushort *)copied_indices)[i] = idx;
                        indicesp += 2;
                        break;
                    default:
                        break;
                }
                if (first < 0 || idx < first) {
                    first = idx;
                }
                if (last < 0 || idx > last) {
                    last = idx;
                }
            }
            
        }
    } else {
        gles2_ebo* ebo = gles2_ebo_find(buffer, ctx->ebo_list);
        copied_indices = (void *)(intptr_t)indicesp;
        if(ebo) {
            if(type==GL_UNSIGNED_BYTE) {
                first = ebo->b_min;
                last = ebo->b_max;
            } else if(type == GL_UNSIGNED_SHORT) {
                first= ebo->s_min;
                last = ebo->s_max;
            }
        } else {
            GLES2_PRINT("Element array buffer %d is binded but was not found in local ebo list!\n", buffer);
        }
    }

    gles2_TransferArrays(s, ctx, first, last - first + 1);
    GLES2_BARRIER_ARG;

    hgl.glDrawElements(mode, count, type, copied_indices);
    if (!buffer && copied_indices) {
        free(copied_indices);
    }
    GLES2_BARRIER_RET;
}

GLES2_CB(glEnable)
{
    GLES2_ARG(TGLenum, cap);
    GLES2_BARRIER_ARG;

    hgl.glEnable(cap);

    GLES2_BARRIER_RET;
}

GLES2_CB(glFinish)
{
    // Important to do this way, so that we don't return too early.
    GLES2_BARRIER_ARG;
    hgl.glFinish();
    GLES2_BARRIER_RET;
}

GLES2_CB(glFlush)
{
    GLES2_BARRIER_ARG_NORET;
    hgl.glFlush();
}

GLES2_CB(glFrontFace)
{
    GLES2_ARG(TGLenum, mode);
    GLES2_BARRIER_ARG_NORET;

    hgl.glFrontFace(mode);
}

GLES2_CB(glGenBuffers)
{
    GLES2_ARG(TGLsizei, n);
    GLES2_ARG(Tptr, buffersp);
    GLES2_BARRIER_ARG;

    GLuint *buffers = NULL;
    if (n > 0) {
        buffers = malloc(n * sizeof(GLuint));
    }
    hgl.glGenBuffers(n, buffers);
    
    GLES2_BARRIER_RET;
    if (buffers) {
        GLsizei i;
        for (i = 0; i < n; i++, buffersp += sizeof(TGLuint)) {
            gles2_put_TGLuint(s, buffersp, buffers[i]);
        }
        free(buffers);
    }
}

GLES2_CB(glGenTextures)
{
    GLES2_ARG(TGLsizei, n);
    GLES2_ARG(Tptr, texturesp);
    GLES2_BARRIER_ARG;

    GLuint* textures = NULL;
    if (n > 0) {
        textures = malloc(sizeof(GLuint)*n);
    }
    hgl.glGenTextures(n, textures);
    GLES2_BARRIER_RET;
    if (textures) {
        GLsizei i;
        for(i = 0; i < n; ++i) {
            gles2_put_TGLuint(s, texturesp + i*sizeof(TGLuint), textures[i]);
        }
        free(textures);
    }
}

GLES2_CB(glGetBooleanv)
{
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);
    GLES2_BARRIER_ARG;

    GLboolean params[16];
    hgl.glGetBooleanv(pname, params);
    unsigned const count = gles2_GetCount(pname);
    unsigned i;
    GLES2_BARRIER_RET;
    for(i = 0; i < count; ++i) {
        gles2_put_TGLboolean(s, paramsp + i*sizeof(TGLboolean), params[i]);
    }
}

GLES2_CB(glGetBufferParameteriv)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, value);
    GLES2_ARG(Tptr, datap);
    GLES2_BARRIER_ARG;

    GLint data = 0;
    hgl.glGetBufferParameteriv(target, value, &data);

    GLES2_BARRIER_RET;
    gles2_put_TGLint(s, datap, data);
}

GLES2_CB(glGetError)
{
    GLES2_BARRIER_ARG;

    GLenum error = hgl.glGetError();

    GLES2_BARRIER_RET;

    if (error != GL_NO_ERROR) {
        fprintf(stderr, "WARNING: Previous call to function %s caused an error 0x%x!\n",
            c->prev_call->name, error);
    }
    gles2_ret_TGLenum(s, error);
}

GLES2_CB(glGetFloatv)
{
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);
    GLES2_BARRIER_ARG;

    GLfloat params[16];
    hgl.glGetFloatv(pname, params);
    unsigned const count = gles2_GetCount(pname);
    unsigned i;
    GLES2_BARRIER_RET;
    for(i = 0; i < count; ++i) {
        gles2_put_TGLfloat(s, paramsp + i*sizeof(TGLfloat), params[i]);
    }
}

GLES2_CB(glGetIntegerv)
{
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);
    GLES2_BARRIER_ARG;

    GLint params[16];
    hgl.glGetIntegerv(pname, params);
    unsigned const count = gles2_GetCount(pname);
    unsigned i;
    GLES2_BARRIER_RET;
    for(i = 0; i < count; ++i) {
        gles2_put_TGLint(s, paramsp + i*sizeof(TGLint), params[i]);
    }
}

//FIXME: couldnt find glGetString implementation

GLES2_CB(glGetTexParameterfv)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);
    GLES2_BARRIER_ARG;

    GLfloat params[4];
    hgl.glGetTexParameterfv(target, pname, params);
    unsigned const count = gles2_glTexParameterCount(pname);
    unsigned i;
    GLES2_BARRIER_RET;
    for(i = 0; i < count; ++i) {
        gles2_put_TGLfloat(s, paramsp + i*sizeof(TGLfloat), params[i]);
    }
}

GLES2_CB(glGetTexParameteriv)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);
    GLES2_BARRIER_ARG;

    GLint params[4];
    hgl.glGetTexParameteriv(target, pname, params);
    unsigned const count = gles2_glTexParameterCount(pname);
    unsigned i;
    GLES2_BARRIER_RET;
    for(i = 0; i < count; ++i) {
        gles2_put_TGLint(s, paramsp + i*sizeof(TGLint), params[i]);
    }
}

GLES2_CB(glHint)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, mode);
    GLES2_BARRIER_ARG_NORET;

    if(s->quality <= 75)
    {
        switch(target)
        {
            default: mode = GL_FASTEST; break;
        }
    }

    hgl.glHint(target, mode);
}

GLES2_CB(glIsBuffer)
{
    GLES2_ARG(TGLuint, buffer);
    GLES2_BARRIER_ARG;

    GLES2_BARRIER_RET;
    gles2_ret_TGLboolean(s, hgl.glIsBuffer(buffer));
}

GLES2_CB(glIsEnabled)
{
    GLES2_ARG(TGLenum, cap);
    GLES2_BARRIER_ARG;

    GLES2_BARRIER_RET;
    gles2_ret_TGLboolean(s, hgl.glIsEnabled(cap));
}

GLES2_CB(glIsTexture)
{
    GLES2_ARG(TGLuint, texture);
    GLES2_BARRIER_ARG;

    GLES2_BARRIER_RET;
    gles2_ret_TGLboolean(s, hgl.glIsTexture(texture));
}

GLES2_CB(glLineWidth)
{
    GLES2_ARG(TGLfloat, width);
    GLES2_BARRIER_ARG_NORET;

    hgl.glLineWidth(width);
}

GLES2_CB(glPixelStorei)
{
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(TGLint, param);
    GLES2_BARRIER_ARG_NORET;

    hgl.glPixelStorei(pname, param);
}

GLES2_CB(glPolygonOffset)
{
    GLES2_ARG(TGLfloat, factor);
    GLES2_ARG(TGLfloat, units);
    GLES2_BARRIER_ARG_NORET;

    hgl.glPolygonOffset(factor, units);
}

GLES2_CB(glReadPixels)
{
    GLES2_ARG(TGLint, x);
    GLES2_ARG(TGLint, y);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLenum, format);
    GLES2_ARG(TGLenum, type);
    GLES2_ARG(Tptr, pixelsp);
    GLES2_BARRIER_ARG;

    char* pixels = NULL;
    unsigned nbytes = 0;
    if (width > 0 && height > 0) {
        unsigned bpp;
        switch (format) {
            case GL_ALPHA: bpp = 1; break;
            case GL_RGB: bpp = (type == GL_UNSIGNED_BYTE) ? 3 : 2; break;
            case GL_RGBA: bpp = (type == GL_UNSIGNED_BYTE) ? 4 : 2; break;
            case GL_LUMINANCE: bpp = 1; break;
            case GL_LUMINANCE_ALPHA: bpp = 2; break;
            default:
                GLES2_PRINT("ERROR: Unknown format 0x%x...\n", format);
                bpp = 1;
                break;
        }

        GLES2_PRINT("Reading %dx%dx%d image at %d,%d...\n",
                    width, height, bpp, x, y);
        nbytes = width*height*bpp;
        pixels = malloc(nbytes);
    }
    hgl.glReadPixels(x, y, width, height, format, type, pixels);
    GLES2_BARRIER_RET;
    if (pixels) {
        gles2_transfer(s, pixelsp, nbytes, pixels, 1);
        free(pixels);
    }
}

GLES2_CB(glSampleCoverage)
{
    GLES2_ARG(TGLclampf, value);
    GLES2_ARG(TGLboolean, invert);
    GLES2_BARRIER_ARG_NORET;

    hgl.glSampleCoverage(value, invert);
}

GLES2_CB(glScissor)
{
    GLES2_ARG(TGLint, x);
    GLES2_ARG(TGLint, y);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_BARRIER_ARG_NORET;

    hgl.glScissor(x, y, width, height);
}

GLES2_CB(glStencilFunc)
{
    GLES2_ARG(TGLenum, func);
    GLES2_ARG(TGLint, ref);
    GLES2_ARG(TGLuint, mask);
    GLES2_BARRIER_ARG_NORET;

    hgl.glStencilFunc(func, ref, mask);
}

GLES2_CB(glStencilMask)
{
    GLES2_ARG(TGLuint, mask);
    GLES2_BARRIER_ARG_NORET;

    hgl.glStencilMask(mask);
}

GLES2_CB(glStencilOp)
{
    GLES2_ARG(TGLenum, fail);
    GLES2_ARG(TGLenum, zfail);
    GLES2_ARG(TGLenum, zpass);
    GLES2_BARRIER_ARG_NORET;

    hgl.glStencilOp(fail, zfail, zpass);
}

GLES2_CB(glTexImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLint, internalformat);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLint, border);
    GLES2_ARG(TGLenum, format);
    GLES2_ARG(TGLenum, type);
    GLES2_ARG(Tptr, pixelsp);

    unsigned bpp;

    switch(format) {
        case GL_ALPHA: bpp = 1; break;
        case GL_RGB: bpp = (type == GL_UNSIGNED_BYTE) ? 3 : 2; break;
        case GL_RGBA: bpp = (type == GL_UNSIGNED_BYTE) ? 4 : 2; break;
        case GL_LUMINANCE: bpp = 1; break;
        case GL_LUMINANCE_ALPHA: bpp = 2; break;
        default:
            GLES2_PRINT("ERROR: Unknown format 0x%x...\n", format);
            bpp = 1;
            break;
    }

    GLES2_PRINT("Uploading %dx%dx%d image...\n", width, height, bpp);
    char* pixels = NULL;
    if (pixelsp && width > 0 && height > 0) {
        TGLsizei nbytes = width*height*bpp;
        pixels = malloc(nbytes);
        gles2_transfer(s, pixelsp, nbytes, pixels, 0);
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glTexImage2D(target, level, internalformat, width, height, border, format, type, pixels);
    if (pixels) {
        free(pixels);
    }
}

GLES2_CB(glTexParameterf)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(TGLfloat, param);
    GLES2_BARRIER_ARG_NORET;

    hgl.glTexParameterf(target, pname, param);
}

GLES2_CB(glTexParameterfv)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);

    GLfloat params[4];
    unsigned const count = gles2_glTexParameterCount(pname);
    unsigned i;
    for (i = 0; i < count; ++i) {
        params[i] = gles2_get_TGLfloat(s, paramsp + i*sizeof(TGLfloat));
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glTexParameterfv(target, pname, params);
}

GLES2_CB(glTexParameteri)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(TGLint, param);
    GLES2_BARRIER_ARG_NORET;

    if(s->quality <= 50)
    {
        switch(pname)
        {
            case GL_TEXTURE_MIN_FILTER: param = GL_NEAREST; break;
            case GL_TEXTURE_MAG_FILTER: param = GL_NEAREST; break;
            default: break;
        }
    }

    hgl.glTexParameteri(target, pname, param);
}

GLES2_CB(glTexParameteriv)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLenum, pname);
    GLES2_ARG(Tptr, paramsp);

    GLint params[4];
    unsigned const count = gles2_glTexParameterCount(pname);
    unsigned i;
    for(i = 0; i < count; ++i) {
        params[i] = gles2_get_TGLint(s, paramsp + i*sizeof(GLint));
    }
    GLES2_BARRIER_ARG_NORET;

    hgl.glTexParameteriv(target, pname, params);
}

GLES2_CB(glTexSubImage2D)
{
    GLES2_ARG(TGLenum, target);
    GLES2_ARG(TGLint, level);
    GLES2_ARG(TGLint, xoffset);
    GLES2_ARG(TGLint, yoffset);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_ARG(TGLenum, format);
    GLES2_ARG(TGLenum, type);
    GLES2_ARG(Tptr, pixelsp);

    unsigned bpp;
    switch (format) {
        case GL_ALPHA: bpp = 1; break;
        case GL_RGB: bpp = (type == GL_UNSIGNED_BYTE) ? 3 : 2; break;
        case GL_RGBA: bpp = (type == GL_UNSIGNED_BYTE) ? 4 : 2; break;
        case GL_LUMINANCE: bpp = 1; break;
        case GL_LUMINANCE_ALPHA: bpp = 2; break;
        default:
            GLES2_PRINT("ERROR: Unknown format 0x%x...\n", format);
            bpp = 1;
            break;
    }

    GLES2_PRINT("Uploading partial %dx%dx%d image at %d,%d (ptr=0x%x)...\n",
    width, height, bpp, xoffset, yoffset, pixelsp);

    char *pixels = NULL;
    if (pixelsp && width > 0 && height > 0) {
        TGLsizei nbytes = width*height*bpp;
        pixels = malloc(nbytes);
        gles2_transfer(s, pixelsp, nbytes, pixels, 0);
    }
    GLES2_BARRIER_ARG_NORET;
    hgl.glTexSubImage2D(target, level, xoffset, yoffset, width, height, format, type, pixels);
    if (pixels) {
        free(pixels);
    }
}

GLES2_CB(glViewport)
{
    GLES2_ARG(TGLint, x);
    GLES2_ARG(TGLint, y);
    GLES2_ARG(TGLsizei, width);
    GLES2_ARG(TGLsizei, height);
    GLES2_BARRIER_ARG_NORET;

    hgl.glViewport(x, y, width, height);
}
