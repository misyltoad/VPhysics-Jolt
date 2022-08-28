
#pragma once

class JoltPhysicsFrictionSnapshot final : public IPhysicsFrictionSnapshot
{
public:
	bool IsValid() override;

	IPhysicsObject *GetObject( int index ) override;
	int GetMaterial( int index ) override;

	void GetContactPoint( Vector &out ) override;

	void GetSurfaceNormal( Vector &out ) override;
	float GetNormalForce() override;
	float GetEnergyAbsorbed() override;

	void RecomputeFriction() override;
	void ClearFrictionForce() override;

	void MarkContactForDelete() override;
	void DeleteAllMarkedContacts( bool wakeObjects ) override;

	void NextFrictionData() override;
	float GetFrictionCoefficient() override;
};
