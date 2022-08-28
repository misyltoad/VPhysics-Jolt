
#pragma once

// This function fixes up the base object
// after loading a single KV value.
using JoltKVSchemaFixupFunc_t = void (*)( void *pBaseObject );

struct JoltKVSchemaFunc_t
{
	size_t ptr_size;
	void ( *ReadFunc )( KeyValues *pProp, void *pPtr, size_t size );
};

struct JoltKVSchemaProp_t
{
	const char				*pszName;
	size_t					offset;
	size_t					size;
	size_t					arrayOffset;
	JoltKVSchemaFunc_t		func;

	JoltKVSchemaFixupFunc_t fixupFunc;
};

struct JoltPhysicsIntPair
{
	int Index0;
	int Index1;
};

extern const JoltKVSchemaFunc_t FillBaseProp;
extern const JoltKVSchemaFunc_t FillStringProp;
extern const JoltKVSchemaFunc_t FillIntProp;
extern const JoltKVSchemaFunc_t FillFloatProp;
extern const JoltKVSchemaFunc_t FillUnsignedCharProp;
extern const JoltKVSchemaFunc_t FillBoolProp;
extern const JoltKVSchemaFunc_t FillVectorProp;
extern const JoltKVSchemaFunc_t FillVector4DProp;
extern const JoltKVSchemaFunc_t FillQAngleProp;
extern const JoltKVSchemaFunc_t FillIntPairProp;
extern const JoltKVSchemaFunc_t FillGameMaterialProp;
extern const JoltKVSchemaFunc_t FillSurfaceProp;
extern const JoltKVSchemaFunc_t FillSoundProp;

#define KVSCHEMA_DESC_ARRAY( type, x, len ) \
	offsetof( type, x ), sizeof( *type::x ), offsetof( type, len )

#define KVSCHEMA_DESC( type, x ) \
	offsetof( type, x ), sizeof( type::x ), static_cast<size_t>(~0llu)

#define KVSCHEMA_DESC_NO_OFFSET( type ) \
	0, sizeof( type ), static_cast<size_t>(~0llu)

void ParseJoltKVSchema( KeyValues *pKV, const JoltKVSchemaProp_t *pDescs, uint count, void *pObj, void *pUnknownKeyObj = nullptr, IVPhysicsKeyHandler* pUnknownKeyHandler = nullptr );
void ParseJoltKVCustom( KeyValues *pKV, void *pUnknownKeyObj, IVPhysicsKeyHandler* pUnknownKeyHandler );
KeyValues *HeaderlessKVBufferToKeyValues( const char *pszBuffer, const char *pszSetName );
