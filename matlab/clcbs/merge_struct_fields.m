function outStruct = merge_struct_fields(baseStruct, patchStruct)
% merge_struct_fields
% Recursively overwrite baseStruct fields with patchStruct fields.

    outStruct = baseStruct;
    if isempty(patchStruct)
        return;
    end

    patchFields = fieldnames(patchStruct);
    for i = 1:numel(patchFields)
        f = patchFields{i};
        if isstruct(patchStruct.(f)) && isfield(outStruct, f) && isstruct(outStruct.(f))
            outStruct.(f) = merge_struct_fields(outStruct.(f), patchStruct.(f));
        else
            outStruct.(f) = patchStruct.(f);
        end
    end
end
